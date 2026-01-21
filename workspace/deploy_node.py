import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point
from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_interfaces.srv import Takeoff, Land
from builtin_interfaces.msg import Duration
import numpy as np
from stable_baselines3 import PPO
import os
import yaml

# ============================================================
# RAYCASTING (Copied from rl.py)
# ============================================================
def raycast(grid, origin, pos, direction, res, max_range):
    steps = int(max_range / res)
    x, y = pos

    for i in range(steps):
        x += direction[0] * res
        y += direction[1] * res

        gx = int((x - origin[0]) / res)
        gy = int((y - origin[1]) / res)

        if gx < 0 or gy < 0 or gx >= grid.shape[1] or gy >= grid.shape[0]:
            return i * res

        if grid[gy, gx] == 1:
            return i * res

    return max_range

class RLInferenceNode(Node):
    def __init__(self):
        super().__init__('rl_inference_node')

        # --- Parameters ---
        self.declare_parameter('model_path', '/root/workspace/src/rl_test/models/ppo_drone_v3.zip')
        self.declare_parameter('grid_path', '/root/workspace/src/rl_test/occupancy_grid_z1.csv')
        self.declare_parameter('meta_path', '/root/workspace/src/rl_test/grid_metadata.txt')
        self.declare_parameter('config_path', '/root/workspace/src/rl_test/config.yaml')
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        grid_path = self.get_parameter('grid_path').get_parameter_value().string_value
        meta_path = self.get_parameter('meta_path').get_parameter_value().string_value
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        # --- Load Data ---
        self.get_logger().info(f"Loading grid from {grid_path}...")
        self.grid = np.loadtxt(grid_path, delimiter=",").astype(np.uint8)
        
        self.meta = {}
        with open(meta_path) as f:
            for line in f:
                k, v = line.split()
                self.meta[k] = float(v)
        
        self.res = self.meta["resolution"]
        self.origin = np.array([self.meta["origin_x"], self.meta["origin_y"]])

        # --- Load Config ---
        self.get_logger().info(f"Loading config from {config_path}...")
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)
            
        self.goal_speed = self.cfg.get('goal_speed', 1.0)
        self.prev_action = np.zeros(2) # For smoothing

        # --- Load Model ---
        self.get_logger().info(f"Loading PPO model from {model_path}...")
        self.model = PPO.load(model_path)

        # --- State ---
        self.current_pos = None
        self.current_vel = None
        self.goal = np.array([9.0, -3.0], dtype=float)
        self.prev_goal = None
        self.goal_vel = np.array([0.0, 0.0])
        
        # Raycasting setup
        self.n_rays = self.cfg.get('n_rays', 8)
        self.max_range = self.cfg.get('max_range', 5.0)
        self.ray_angles = np.linspace(0, 2 * np.pi, self.n_rays, endpoint=False)

        # --- ROS Interfaces ---
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/cf_1/pose',
            self.pose_cb,
            10
        )
        
        self.vel_sub = self.create_subscription(
            LogDataGeneric,
            '/cf_1/velocity',
            self.vel_cb,
            10
        )

        self.goal_sub = self.create_subscription(
            Point,
            '/AGV/pose',
            self.goal_cb,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cf_1/cmd_vel', 10)

        # Takeoff Client
        self.takeoff_client = self.create_client(Takeoff, '/cf_1/takeoff')
        self.land_client = self.create_client(Land, '/cf_1/land')
        self.is_flying = False
        
        # Start takeoff sequence
        self.takeoff_timer = self.create_timer(1.0, self.trigger_takeoff)

        # Control loop at 10Hz (dt=0.1)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("RL Inference Node Started")

    def trigger_takeoff(self):
        if not self.takeoff_client.service_is_ready():
            self.get_logger().info("Waiting for takeoff service...")
            return
            
        self.takeoff_timer.cancel()
        self.get_logger().info("Taking off...")
        
        req = Takeoff.Request()
        req.height = 1.0
        req.duration = Duration(sec=2, nanosec=0)
        req.group_mask = 0
        
        future = self.takeoff_client.call_async(req)
        future.add_done_callback(self.on_takeoff_complete)

    def on_takeoff_complete(self, future):
        try:
            future.result()
            self.get_logger().info("Takeoff complete. Starting RL control.")
            self.is_flying = True
        except Exception as e:
            self.get_logger().error(f"Takeoff failed: {e}")

    def trigger_land(self):
        # 1. Immediately stop the RL control loop logic
        self.is_flying = False 
        
        if not self.land_client.service_is_ready():
            self.get_logger().info("Waiting for land service...")
            return

        # 2. Send a "Stop" command to clear any high-velocity buffers
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

        self.get_logger().info("Requesting Land...")
        req = Land.Request()
        req.height = 0.0
        req.duration = Duration(sec=3, nanosec=0)
        req.group_mask = 0
        
        # Use a future to track result
        self.land_client.call_async(req).add_done_callback(self.on_land_complete)

    def on_land_complete(self, future):
        try:
            future.result()
            self.get_logger().info("Land service called successfully.")
        except Exception as e:
            self.get_logger().error(f"Land failed: {e}")

    def pose_cb(self, msg):
        self.current_pos = np.array([msg.pose.position.x, msg.pose.position.y])

    def vel_cb(self, msg):
        # LogDataGeneric has .values list
        # Assuming [vx, vy, vz]
        if len(msg.values) >= 2:
            self.current_vel = np.array([msg.values[0], msg.values[1]])

    def goal_cb(self, msg):
        new_goal = np.array([msg.x, msg.y])
        
        # Calculate velocity: (current - previous) / dt
        if self.prev_goal is not None:
            # dt is 0.1 because your control loop/timer runs at 10Hz
            self.goal_vel = (new_goal - self.prev_goal) / 0.1
            
        self.prev_goal = new_goal
        self.goal = new_goal
        self.get_logger().info(f"New goal received: {self.goal}, Vel: {self.goal_vel}")

    def control_loop(self):
        if not self.is_flying:
            return
            
        if self.current_pos is None or self.current_vel is None:
            return

        rel_pos = self.goal - self.current_pos
        dist_to_goal = np.linalg.norm(rel_pos)

        # if dist_to_goal < 0.3:
        #     self.get_logger().info(f"Goal reached! Dist: {dist_to_goal:.2f}. Triggering landing.")
        #     self.trigger_land()
        #     return # Exit early so we don't predict/publish this cycle
        
        # Debug logging every 10 steps (approx 1 sec)
        # We can use a counter or just print every time for now since it's debugging
        # self.get_logger().info(f"Dist: {dist_to_goal:.2f}, Pos: {self.current_pos}")
        
        rays = []
        for angle in self.ray_angles:
            direction = np.array([np.cos(angle), np.sin(angle)])
            dist = raycast(self.grid, self.origin, self.current_pos, direction, self.res, self.max_range)
            rays.append(dist)

        # Normalize rays as per training
        norm_rays = [r / self.max_range for r in rays]

        obs = np.array([
            rel_pos[0],
            rel_pos[1],
            self.current_vel[0],
            self.current_vel[1],
            self.goal_vel[0], # Added
            self.goal_vel[1]  # Added
        ] + norm_rays, dtype=np.float32)

        # --- Predict ---
        action, _ = self.model.predict(obs, deterministic=True)
        
        # --- Action Smoothing ---
        alpha = 0.5
        filtered_action = alpha * action + (1 - alpha) * self.prev_action
        self.prev_action = filtered_action
        
        # --- Publish Command ---
        cmd = Twist()
        # Scale action from [-1, 1] to target speed
        cmd.linear.x = float(filtered_action[0]) * 0.6
        cmd.linear.y = float(filtered_action[1]) * 0.6
        
        # Z velocity and Yaw rate are 0 for this 2D navigation task
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RLInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
