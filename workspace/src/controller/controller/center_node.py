#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from crazyflie_interfaces.srv import Takeoff, GoTo, Land
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

class CenterDrone(Node):
    def __init__(self, drone_name, target_x, target_y, target_z):
        """
        Logic for the Center Drone: Takeoff to target_z and move to target_x, target_y.
        """
        super().__init__(f'{drone_name}_center_logic')
        
        # Configuration
        self.cf_id = drone_name
        self.target_pos = [float(target_x), float(target_y), float(target_z)]
        self.velocity = 0.2
        self.current_pose = [0.0, 0.0, 0.0] # x, y, z

        # Clients
        self.takeoff_cli = self.create_client(Takeoff, f'/{self.cf_id}/takeoff')
        self.goto_cli = self.create_client(GoTo, f'/{self.cf_id}/go_to')
        self.land_cli = self.create_client(Land, f'/{self.cf_id}/land')

        # Subscriptions
        self.create_subscription(PoseStamped, f'/{self.cf_id}/pose', self._pose_callback, 10)
        
        self.get_logger().info(f"Center Node for {self.cf_id} initialized.")

    def _pose_callback(self, msg):
        self.current_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def wait_for_services(self):
        self.get_logger().info(f"[{self.cf_id}] Waiting for services...")
        self.takeoff_cli.wait_for_service()
        self.goto_cli.wait_for_service()

    def run_center_mission(self):
        """
        The main logic function to be called from the integrated file.
        """
        self.wait_for_services()

        # 1. Takeoff to target altitude
        self.get_logger().info(f"[{self.cf_id}] Taking off to {self.target_pos[2]}m")
        tk_req = Takeoff.Request()
        tk_req.height = self.target_pos[2]
        tk_req.duration = rclpy.duration.Duration(seconds=3.0).to_msg()
        self.takeoff_cli.call_async(tk_req)
        time.sleep(4.0)

        # 2. Calculate distance and duration for horizontal move
        dist = math.sqrt((self.target_pos[0] - self.current_pose[0])**2 + 
                         (self.target_pos[1] - self.current_pose[1])**2)
        duration = max(dist / self.velocity, 4.0)

        # 3. Move to Center Coordinates
        self.get_logger().info(f"[{self.cf_id}] Moving to ({self.target_pos[0]}, {self.target_pos[1]})")
        gt_req = GoTo.Request()
        gt_req.goal.x, gt_req.goal.y, gt_req.goal.z = self.target_pos
        gt_req.yaw = 0.0
        gt_req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        gt_req.relative = False
        
        self.goto_cli.call_async(gt_req)
        time.sleep(duration + 1.0)
        
        self.get_logger().info(f"[{self.cf_id}] Center Position Reached.")