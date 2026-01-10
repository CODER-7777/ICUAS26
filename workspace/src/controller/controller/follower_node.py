#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from crazyflie_interfaces.srv import GoTo, Takeoff
from geometry_msgs.msg import Point

class FollowerDrone(Node):
    def __init__(self, drone_name, target_z, velocity):
        """
        Follower Logic: Directly tracks /AGV/pose via Async Overstreaming.
        """
        super().__init__(f'{drone_name}_follower_logic')
        
        self.cf_id = drone_name
        self.follow_height = float(target_z)
        self.velocity = float(velocity)
        
        # AGV State
        self.current_agv_pose = None

        # Service Clients
        self.goto_cli = self.create_client(GoTo, f'/{self.cf_id}/go_to')
        self.takeoff_cli = self.create_client(Takeoff, f'/{self.cf_id}/takeoff')

        # Subscriber to AGV Point topic
        self.create_subscription(Point, '/AGV/pose', self._agv_pose_callback, 10)
        
        self.get_logger().info(f"Follower {self.cf_id} ready to follow AGV.")

    def _agv_pose_callback(self, msg):
        """Simple update of the AGV position."""
        self.current_agv_pose = msg

    def wait_for_services(self):
        self.get_logger().info(f"[{self.cf_id}] Connecting to services...")
        self.takeoff_cli.wait_for_service()
        self.goto_cli.wait_for_service()

    def run_follower_mission(self):
        """
        Main execution loop.
        """
        self.wait_for_services()

        # 1. Takeoff
        self.get_logger().info(f"[{self.cf_id}] Taking off...")
        tk_req = Takeoff.Request()
        tk_req.height = self.follow_height
        tk_req.duration = rclpy.duration.Duration(seconds=3.0).to_msg()
        self.takeoff_cli.call_async(tk_req)
        time.sleep(4.0)

        # 2. High-Frequency Following Loop
        self.get_logger().info(f"[{self.cf_id}] Tracking AGV at 10Hz.")
        
        while rclpy.ok():
            if self.current_agv_pose is not None:
                # Prepare the request
                req = GoTo.Request()
                req.goal.x = float(self.current_agv_pose.x)
                req.goal.y = float(self.current_agv_pose.y)
                req.goal.z = float(self.follow_height)
                req.yaw = 0.0
                
                # Async Smoothing: Duration (0.2s) > Update Rate (0.1s)
                # This prevents the drone from stopping between updates.
                req.duration = rclpy.duration.Duration(seconds=0.2).to_msg()
                req.relative = False
                
                # ASYNC CALL: Crucial to prevent blocking the 10Hz loop
                self.goto_cli.call_async(req)

            # Match AGV 10Hz frequency
            time.sleep(0.1)
