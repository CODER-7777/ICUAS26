import math
import time
import rclpy
from crazyflie_interfaces.srv import GoTo

class CenterTask:
    def __init__(self, node, cf_name, target_altitude=5.0, velocity=0.2):
        self.node = node
        self.cf_name = cf_name
        self.target_altitude = target_altitude
        self.velocity = velocity
        
        # Service Client for this specific drone
        self.client_goto = node.create_client(GoTo, f'/{cf_name}/go_to')
        while not self.client_goto.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'Waiting for /{cf_name}/go_to service...')

    def execute(self, current_pose):
        """Perform the actual flight sequence."""
        if current_pose is None:
            return

        start_x, start_y, start_z, _ = current_pose
        self.node.get_logger().info(f"[{self.cf_name}] Starting Center Mission...")

        # 1. Rise Vertically
        vert_dist = abs(self.target_altitude - start_z)
        duration_up = max(vert_dist / self.velocity, 4.0)
        self._send_goto(start_x, start_y, self.target_altitude, 0.0, duration_up)

        # 2. Transition to Center (0, 0)
        horiz_dist = math.sqrt(start_x**2 + start_y**2)
        duration_move = max(horiz_dist / self.velocity, 5.0)
        self.node.get_logger().info(f"[{self.cf_name}] Transitioning to Map Center")
        self._send_goto(0.0, 0.0, self.target_altitude, 0.0, duration_move)

    def _send_goto(self, x, y, z, yaw, duration):
        req = GoTo.Request()
        req.goal.x, req.goal.y, req.goal.z = float(x), float(y), float(z)
        req.yaw = float(yaw)
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.relative = False
        
        self.client_goto.call_async(req)
        time.sleep(duration + 0.5)