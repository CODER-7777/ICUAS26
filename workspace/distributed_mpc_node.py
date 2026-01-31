#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from crazyflie_interfaces.srv import Takeoff, GoTo
from builtin_interfaces.msg import Duration
import numpy as np
import pandas as pd
import cv2
from shapely.geometry import LineString, Polygon
from cvxopt import matrix, solvers
import os

# --- Constants & Helpers ---

def extract_obstacle_vertices(csv_path):
    if not os.path.exists(csv_path): return []
    grid = pd.read_csv(csv_path, header=None).values.astype(np.uint8)
    contours, _ = cv2.findContours(grid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacles_vertices = []
    resolution, origin_x, origin_y = 0.1, 0.0, 0.0
    try:
        with open("grid_metadata.txt", "r") as f:
            for line in f:
                key, val = line.split()
                if key == 'resolution': resolution = float(val)
                elif key == 'origin_x': origin_x = float(val)
                elif key == 'origin_y': origin_y = float(val)
    except FileNotFoundError: pass
    for cnt in contours:
        hull = cv2.convexHull(cnt) 
        if len(hull) < 3: continue
        vertices_m = [[pt[0][0] * resolution + origin_x, pt[0][1] * resolution + origin_y] for pt in hull]
        obstacles_vertices.append(np.array(vertices_m))
    return obstacles_vertices

class MiniEdgeRRTStar:
    def __init__(self, start, goal, obstacles, dc):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = [Polygon(v) for v in obstacles]
        self.dc = dc
        self.kappa = 500.0
        self.tree = {0: {'pos': self.start, 'parent': None, 'cost': 0}}

    def plan(self, max_iter=3000):
        for _ in range(max_iter):
            rand_pt = np.random.uniform(-10, 10, size=2)
            best_parent, min_c = None, float('inf')
            for node_id, node in self.tree.items():
                d = np.linalg.norm(node['pos'] - rand_pt)
                if d <= self.dc:
                    cost = node['cost'] + d + self.kappa
                    if cost < min_c and self.is_clear(node['pos'], rand_pt):
                        min_c, best_parent = cost, node_id
            if best_parent is not None:
                new_id = len(self.tree)
                self.tree[new_id] = {'pos': rand_pt, 'parent': best_parent, 'cost': min_c}
                if np.linalg.norm(rand_pt - self.goal) <= self.dc:
                    if self.is_clear(rand_pt, self.goal): return self.extract_path(new_id)
        return None

    def is_clear(self, p1, p2):
        line = LineString([p1, p2])
        return not any(line.intersects(obs) and line.intersection(obs).length > 1e-4 for obs in self.obstacles)

    def extract_path(self, end_id):
        path, curr = [self.goal], end_id
        while curr is not None:
            path.append(self.tree[curr]['pos']); curr = self.tree[curr]['parent']
        return path[::-1]

def get_separating_plane(agent_pos, obstacle_vertices):
    dists = [np.linalg.norm(agent_pos - v) for v in obstacle_vertices]
    closest_pt = obstacle_vertices[np.argmin(dists)]
    diff = agent_pos - closest_pt
    norm_vec = diff / (np.linalg.norm(diff) + 1e-6)
    offset = np.dot(norm_vec, closest_pt + norm_vec * 0.6)
    return norm_vec, offset

def solve_mpc_step(current_pos, target_ref, obstacles, params):
    K = params['K']
    P = matrix(np.eye(K * 2) * 2.0); q = matrix(-2.0 * np.tile(target_ref, K))
    G_list, h_list = [], []
    for obs in obstacles:
        n, b = get_separating_plane(current_pos, obs)
        G_row = np.zeros((1, K * 2)); G_row[0, :2] = -n
        G_list.append(G_row); h_list.append(-b)
    try:
        solvers.options['show_progress'] = False
        sol = solvers.qp(P, q, matrix(np.vstack(G_list)) if G_list else None, matrix(h_list) if h_list else None)
        return np.array(sol['x'])[:2].flatten()
    except: return current_pos

# --- ROS Node ---

class DistributedMPCNode(Node):
    def __init__(self):
        super().__init__('distributed_mpc_node')
        
        self.num_drones = 5
        self.dc_limit = 3.0
        self.flight_height = 1.0
        self.city_obstacles = extract_obstacle_vertices("occupancy_grid_z1.csv")
        self.mpc_params = {'K': 3, 'h': 0.5, 'dc': self.dc_limit}
        
        self.latest_agv_pose = None
        self.drone_states = None
        self.drones = []
        self.is_running = False
        self.processing_step = False

        for i in range(self.num_drones):
            name = f"cf_{i+1}"
            self.drones.append({
                'name': name,
                'takeoff_client': self.create_client(Takeoff, f'/{name}/takeoff'),
                'goto_client': self.create_client(GoTo, f'/{name}/go_to'),
                'pose_sub': self.create_subscription(Pose, f'/{name}/pose', lambda msg, idx=i: self.pose_callback(msg, idx), 10),
                'current_pose': None
            })

        self.agv_sub = self.create_subscription(Point, '/AGV/pose', self.agv_callback, 10)
        
        # STEP 1: Takeoff
        self.get_logger().info("Phase 1: Drones taking off...")
        self.initiate_takeoff()

    def pose_callback(self, msg, idx):
        self.drones[idx]['current_pose'] = msg

    def agv_callback(self, msg):
        self.latest_agv_pose = np.array([msg.x, msg.y])

    def initiate_takeoff(self):
        for drone in self.drones:
            req = Takeoff.Request()
            req.height = self.flight_height
            req.duration = Duration(sec=3, nanosec=0)
            drone['takeoff_client'].call_async(req)
        
        # After takeoff, wait and then check if we can plan the RRT path
        self.init_timer = self.create_timer(5.0, self.check_ready_to_plan)

    def check_ready_to_plan(self):
        if self.latest_agv_pose is None:
            self.get_logger().info("Waiting for AGV pose to plan RRT chain...")
            return
        self.init_timer.cancel()
        self.plan_and_move_to_initial()

    def plan_and_move_to_initial(self):
        """Phase 2: Use RRT* to find path, then send drones to those points once."""
        self.get_logger().info("Phase 2: Planning RRT* path and moving drones to initial chain positions...")
        
        planner = MiniEdgeRRTStar(start=[0, 0], goal=self.latest_agv_pose, obstacles=self.city_obstacles, dc=self.dc_limit)
        path = planner.plan()
        
        if path is None:
            self.get_logger().warn("RRT* failed. Retrying...")
            self.init_timer = self.create_timer(2.0, self.check_ready_to_plan)
            return

        # Interpolate the RRT path to find 5 drone positions
        points = np.array(path)
        indices = np.linspace(0, len(points) - 1, self.num_drones + 1)
        self.drone_states = points[indices.astype(int)][1:] # [Drone 1, Drone 2, ... Drone 5]

        # Send GoTo for initial positions
        max_travel_time = 0.0
        for i in range(self.num_drones):
            target = self.drone_states[i]
            # Estimate current pos or assume [0,0] if sensors aren't ready
            curr = [0.0, 0.0]
            if self.drones[i]['current_pose']:
                curr = [self.drones[i]['current_pose'].position.x, self.drones[i]['current_pose'].position.y]
            
            dist = np.linalg.norm(target - curr)
            duration = max(dist / 0.5, 3.0) # Move at 0.5m/s, min 3 seconds
            max_travel_time = max(max_travel_time, duration)

            req = GoTo.Request()
            req.goal.x, req.goal.y, req.goal.z = float(target[0]), float(target[1]), self.flight_height
            req.duration = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
            req.relative = False
            self.drones[i]['goto_client'].call_async(req)

        # STEP 3: Wait for drones to arrive at initial spots, then start MPC
        self.get_logger().info(f"Moving to chain positions. Waiting {max_travel_time:.1f}s...")
        self.control_timer = self.create_timer(max_travel_time + 1.0, self.start_mpc_loop)

    def start_mpc_loop(self):
        """Phase 3: Start the continuous MPC tracking loop."""
        self.control_timer.cancel()
        self.get_logger().info("Phase 3: Initial positions reached. Starting MPC control loop.")
        self.is_running = True
        self.control_timer = self.create_timer(0.5, self.control_step)

    def control_step(self):
        if not self.is_running or self.latest_agv_pose is None or self.processing_step:
            return
        self.processing_step = True
        
        prev_drone_states = np.copy(self.drone_states)
        for i in range(self.num_drones):
            if self.drones[i]['current_pose']:
                p = self.drones[i]['current_pose'].position
                prev_drone_states[i] = [p.x, p.y]

        new_states = []
        for i in range(self.num_drones):
            parent = np.array([0.0, 0.0]) if i == 0 else prev_drone_states[i-1]
            child = self.latest_agv_pose if i == self.num_drones - 1 else prev_drone_states[i+1]
            
            ref = (parent + child) / 2.0
            target = solve_mpc_step(prev_drone_states[i], ref, self.city_obstacles, self.mpc_params)
            new_states.append(target)
            
            # Continuous GoTo commands for MPC tracking
            self.send_mpc_goto(i, target, prev_drone_states[i])

        self.drone_states = np.array(new_states)
        self.processing_step = False

    def send_mpc_goto(self, idx, target, current):
        dist = np.linalg.norm(target - current)
        duration = max(dist / 0.5, 1.0) # Allow faster movement (1.0m/s) during MPC
        
        req = GoTo.Request()
        req.goal.x, req.goal.y, req.goal.z = float(target[0]), float(target[1]), self.flight_height
        req.duration = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        req.relative = False
        self.drones[idx]['goto_client'].call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = DistributedMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()