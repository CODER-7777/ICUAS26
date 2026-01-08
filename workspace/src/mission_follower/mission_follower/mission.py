#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import math
import time
import threading

from crazyflie_interfaces.srv import Takeoff, GoTo, Land
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped # To track current position

class MultiCrazyflieWaypointMission(Node):
    def __init__(self):
        super().__init__('multi_cf_waypoint_mission')

        # ===============================
        # CONFIGURATION
        # ===============================
        self.default_velocity = 0.1      # m/s
        self.max_yaw_rate = 0.5          # rad/s (approx 60 deg/sec)
        self.low_battery_threshold = 40.0

        self.drone_csv_map = {
            'cf_1': os.path.expanduser('final_coordinates_cf1.csv'),
            'cf_2': os.path.expanduser('final_coordinates_cf2.csv'),
            'cf_3': os.path.expanduser('final_coordinates_cf3.csv'),
        }

        self.emergency_flags = {'cf_1': False, 'cf_2': False, 'cf_3': False}
        self.home_positions = {'cf_1': [0.5, 0.5, 0.01], 'cf_2': [0.7, 0.7, 0.01], 'cf_3': [0.9, 0.9, 0.01]}
        self.rth_altitudes = {'cf_1': 3.0, 'cf_2': 4.0, 'cf_3': 5.0}
        self.current_poses = {cf: [0.0, 0.0, 0.0, 0.0] for cf in self.drone_csv_map.keys()} # Added Yaw

        self.cf_clients = {}

        for cf in self.drone_csv_map.keys():
            self.cf_clients[cf] = {
                'takeoff': self.create_client(Takeoff, f'/{cf}/takeoff'),
                'goto': self.create_client(GoTo, f'/{cf}/go_to'),
                'land': self.create_client(Land, f'/{cf}/land')
            }
            
            self.create_subscription(BatteryState, f'/{cf}/battery_status', 
                lambda msg, d=cf: self.battery_callback(msg, d), 10)

            self.create_subscription(PoseStamped, f'/{cf}/pose',
                lambda msg, d=cf: self.pose_callback(msg, d), 10)

        self.wait_for_services()

        for cf, csv_path in self.drone_csv_map.items():
            threading.Thread(target=self.execute_mission, args=(cf, csv_path), daemon=True).start()

    # ===============================
    # HELPER: ANGLE WRAPPING
    # ===============================
    def get_yaw_diff(self, target, current):
        """Calculates the shortest distance between two angles in radians."""
        diff = target - current
        while diff > math.pi: diff -= 2.0 * math.pi
        while diff < -math.pi: diff += 2.0 * math.pi
        return abs(diff)

    def pose_callback(self, msg, cf):
        # Extract yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_poses[cf] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            yaw
        ]



    # ===============================
    # CALLBACKS
    # ===============================
    def battery_callback(self, msg, cf):
        p = msg.percentage if msg.percentage > 1.0 else msg.percentage * 100
        if p < self.low_battery_threshold and not self.emergency_flags[cf]:
            self.emergency_flags[cf] = True

    # def pose_callback(self, msg, cf):
    #     self.current_poses[cf] = [
    #         msg.pose.position.x,
    #         msg.pose.position.y,
    #         msg.pose.position.z
    #     ]

    # ===============================
    # SERVICE WAIT
    # ===============================
    def wait_for_services(self):
        self.get_logger().info('Waiting for Crazyflie services...')
        for cf, services in self.cf_clients.items():
            for srv_name, client in services.items():
                while not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f'Waiting for /{cf}/{srv_name}...')

    # ===============================
    # BASIC COMMANDS
    # ===============================
    def send_takeoff(self, cf, height=1.0, duration=3.0):
        req = Takeoff.Request()
        req.height = float(height)
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.cf_clients[cf]['takeoff'].call_async(req)
        time.sleep(duration + 1.0)

    def send_goto(self, cf, x, y, z, yaw, duration):
        req = GoTo.Request()
        req.goal.x, req.goal.y, req.goal.z = float(x), float(y), float(z)
        req.yaw = float(yaw)
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.relative = False
        self.cf_clients[cf]['goto'].call_async(req)
        self.get_logger().info(f'[{cf}] GoTo ({x:.2f}, {y:.2f}, {z:.2f})')
        time.sleep(duration)

    def send_land(self, cf, duration=3.0):
        req = Land.Request()
        req.height = 0.0
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.cf_clients[cf]['land'].call_async(req)
        time.sleep(duration)

    # ==================================
    # REFINED EMERGENCY PROCEDURE
    # ==================================
    # ==================================
    # REFINED EMERGENCY PROCEDURE
    # ==================================
    def perform_emergency_procedure(self, cf):
        home_pos = self.home_positions[cf]
        safe_alt = self.rth_altitudes[cf]
        
        # Get current location for trajectory calculation
        curr_x, curr_y, curr_z = self.current_poses[cf]

        self.get_logger().warn(f'[{cf}] EMERGENCY: Calculating safe return trajectory...')
        
        # 1. Vertical Ascent at current (X,Y) to safety altitude
        vert_dist = abs(safe_alt - curr_z)
        # Dynamic duration: distance / velocity (minimum 2s for stability)
        duration_up = max(vert_dist / self.default_velocity, 4.0)
        
        self.get_logger().info(f'[{cf}] Step 1: Ascending to {safe_alt}m ({duration_up:.1f}s)')
        self.send_goto(cf, curr_x, curr_y, safe_alt, 0.0, duration_up)

        # 2. Horizontal movement to Home (X,Y) at safety altitude
        horiz_dist = math.sqrt((home_pos[0] - curr_x)**2 + (home_pos[1] - curr_y)**2)
        duration_horiz = max(horiz_dist / self.default_velocity, 4.0)
        
        self.get_logger().info(f'[{cf}] Step 2: Flying to Home ({home_pos[0]}, {home_pos[1]}) in {duration_horiz:.1f}s')
        self.send_goto(cf, home_pos[0], home_pos[1], safe_alt, 0.0, duration_horiz)

        # 3. Descent to Landing height (e.g., 0.5m for a soft approach)
        approach_alt = 0.5
        descent_dist = abs(safe_alt - approach_alt)
        duration_down = max(descent_dist / self.default_velocity, 4.0)
        
        self.get_logger().info(f'[{cf}] Step 3: Descending to approach altitude ({duration_down:.1f}s)')
        self.send_goto(cf, home_pos[0], home_pos[1], approach_alt, 0.0, duration_down)

        # 4. Final Land and 5-min wait
        self.send_land(cf)
        self.get_logger().info(f'[{cf}] SAFE. Waiting 5 minutes.')
        time.sleep(300)

    # ===============================
    # MISSION EXECUTION
    # ===============================
    def execute_mission(self, cf, csv_path):
        if not os.path.exists(csv_path): return

        time.sleep(1.0)
        start_pose = self.current_poses[cf]
        
        # Takeoff
        self.send_takeoff(cf, height=1.0)
        
        last_x, last_y, last_z, last_yaw = start_pose[0], start_pose[1], 1.0, start_pose[3]

        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if self.emergency_flags[cf]:
                    self.perform_emergency_procedure(cf)
                    return

                # Parse CSV (Now including yaw)
                x, y, z = float(row['x']), float(row['y']), float(row['z'])
                target_yaw = float(row['yaw']) if 'yaw' in row else 0.0

                # --- OPTIMIZED DURATION CALCULATION ---
                # 1. Travel time (Distance)
                dist = math.sqrt((x-last_x)**2 + (y-last_y)**2 + (z-last_z)**2)
                travel_time = dist / self.default_velocity
                
                # 2. Rotation time (Yaw change)
                yaw_diff = self.get_yaw_diff(target_yaw, last_yaw)
                rotation_time = yaw_diff / self.max_yaw_rate

                # 3. Use the maximum of the two to ensure smooth transition
                duration = max(travel_time, rotation_time, 5)

                self.get_logger().info(f'[{cf}] Moving to ({x:.2f}, {y:.2f}) Yaw: {math.degrees(target_yaw):.1f}°')
                self.send_goto(cf, x, y, z, target_yaw, duration)
                
                last_x, last_y, last_z, last_yaw = x, y, z, target_yaw

        self.send_land(cf)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCrazyflieWaypointMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()