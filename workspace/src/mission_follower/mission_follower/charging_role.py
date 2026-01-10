#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from crazyflie_interfaces.srv import Takeoff, GoTo, Land
from geometry_msgs.msg import PoseStamped


class ChargingTask:
    def __init__(
        self,
        node,
        cf_name,
        cancel_event,
        hover_z=4.0,
        velocity=0.3
    ):
        self.node = node
        self.cf_name = cf_name
        self.cancel_event = cancel_event
        self.hover_z = float(hover_z)
        self.velocity = float(velocity)

        self.current_pose = None

        # Service Clients
        self.takeoff_cli = node.create_client(
            Takeoff, f'/{cf_name}/takeoff'
        )
        self.goto_cli = node.create_client(
            GoTo, f'/{cf_name}/go_to'
        )
        self.land_cli = node.create_client(
            Land, f'/{cf_name}/land'
        )

        # Pose subscription
        self.pose_sub = node.create_subscription(
            PoseStamped,
            f'/{cf_name}/pose',
            self._pose_callback,
            10
        )

        self.node.get_logger().info(
            f"[{cf_name}] ChargingTask initialized"
        )


    def _pose_callback(self, msg):
        self.current_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]


    def execute(self):
        self.node.get_logger().info(
            f"[{self.cf_name}] Charging mission started"
        )

        # Wait for services
        self._wait_for_services()

        # Wait for pose
        while self.current_pose is None and rclpy.ok():
            if self._cancelled():
                return
            time.sleep(0.1)

        start_x, start_y, start_z = self.current_pose

        # ------------------ TAKEOFF ------------------
        if self._cancelled():
            return

        self.node.get_logger().info(
            f"[{self.cf_name}] Taking off to {self.hover_z}m"
        )

        tk_req = Takeoff.Request()
        tk_req.height = self.hover_z
        tk_req.duration = rclpy.duration.Duration(seconds=3.0).to_msg()
        self.takeoff_cli.call_async(tk_req)

        self._sleep_with_cancel(3.5)

        # ------------------ MOVE TO CHARGER ------------------
        if self._cancelled():
            return

        dist_xy = math.sqrt(start_x**2 + start_y**2)
        duration = max(dist_xy / self.velocity, 3.0)

        self.node.get_logger().info(
            f"[{self.cf_name}] Moving to charging station (0,0)"
        )

        gt_req = GoTo.Request()
        gt_req.goal.x = 0.0
        gt_req.goal.y = 0.0
        gt_req.goal.z = self.hover_z
        gt_req.yaw = 0.0
        gt_req.duration = rclpy.duration.Duration(
            seconds=duration
        ).to_msg()
        gt_req.relative = False

        self.goto_cli.call_async(gt_req)
        self._sleep_with_cancel(duration + 0.5)

        # ------------------ LAND ------------------
        if self._cancelled():
            return

        self.node.get_logger().info(
            f"[{self.cf_name}] Landing at charging station"
        )

        ld_req = Land.Request()
        ld_req.height = 0.0
        ld_req.duration = rclpy.duration.Duration(seconds=3.0).to_msg()
        self.land_cli.call_async(ld_req)

        self._sleep_with_cancel(4.0)

        self.node.get_logger().info(
            f"[{self.cf_name}] Charging mission completed"
        )


    # ===================== HELPERS =====================
    def _wait_for_services(self):
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            if self._cancelled():
                return
            self.node.get_logger().info(
                f"[{self.cf_name}] Waiting for takeoff service..."
            )

        while not self.goto_cli.wait_for_service(timeout_sec=1.0):
            if self._cancelled():
                return
            self.node.get_logger().info(
                f"[{self.cf_name}] Waiting for go_to service..."
            )

        while not self.land_cli.wait_for_service(timeout_sec=1.0):
            if self._cancelled():
                return
            self.node.get_logger().info(
                f"[{self.cf_name}] Waiting for land service..."
            )


    def _sleep_with_cancel(self, duration):
        end = time.time() + duration
        while time.time() < end:
            if self._cancelled():
                return
            time.sleep(0.1)


    def _cancelled(self):
        if self.cancel_event.is_set():
            self.node.get_logger().warn(
                f"[{self.cf_name}] Charging mission cancelled"
            )
            return True
        return False
