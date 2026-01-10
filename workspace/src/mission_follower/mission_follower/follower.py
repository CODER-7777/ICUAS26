#!/usr/bin/env python3

import rclpy
import time
from crazyflie_interfaces.srv import GoTo, Takeoff
from geometry_msgs.msg import Point


class FollowerTask:
    def __init__(
        self,
        node,
        cf_name,
        cancel_event,
        follow_height=1.5,
        velocity=0.5
    ):
        """
        FOLLOW role: continuously tracks /AGV/pose at ~10Hz
        """
        self.node = node
        self.cf_name = cf_name
        self.cancel_event = cancel_event

        self.follow_height = float(follow_height)
        self.velocity = float(velocity)

        self.current_agv_pose = None

        # Service clients
        self.takeoff_cli = node.create_client(
            Takeoff, f'/{cf_name}/takeoff'
        )
        self.goto_cli = node.create_client(
            GoTo, f'/{cf_name}/go_to'
        )

        # AGV pose subscriber
        self.pose_sub = node.create_subscription(
            Point,
            '/AGV/pose',
            self._agv_pose_callback,
            10
        )

        self.node.get_logger().info(
            f"[{cf_name}] FollowerTask initialized"
        )

    def _agv_pose_callback(self, msg):
        self.current_agv_pose = msg

    # ===================== MAIN =====================
    def execute(self):
        self.node.get_logger().info(
            f"[{self.cf_name}] FOLLOW mission started"
        )

        self._wait_for_services()

        # ------------------ TAKEOFF ------------------
        if self._cancelled():
            return

        tk_req = Takeoff.Request()
        tk_req.height = self.follow_height
        tk_req.duration = rclpy.duration.Duration(seconds=3.0).to_msg()
        self.takeoff_cli.call_async(tk_req)

        self._sleep_with_cancel(3.5)

        # ------------------ FOLLOW LOOP ------------------
        self.node.get_logger().info(
            f"[{self.cf_name}] Following AGV at 10Hz"
        )

        while rclpy.ok():
            if self._cancelled():
                return

            if self.current_agv_pose is not None:
                req = GoTo.Request()
                req.goal.x = float(self.current_agv_pose.x)
                req.goal.y = float(self.current_agv_pose.y)
                req.goal.z = self.follow_height
                req.yaw = 0.0

                # Smooth continuous tracking
                req.duration = rclpy.duration.Duration(
                    seconds=0.2
                ).to_msg()
                req.relative = False

                # ASYNC → does NOT block 10Hz loop
                self.goto_cli.call_async(req)

            time.sleep(0.1)

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

    def _sleep_with_cancel(self, duration):
        end = time.time() + duration
        while time.time() < end:
            if self._cancelled():
                return
            time.sleep(0.1)

    def _cancelled(self):
        if self.cancel_event.is_set():
            self.node.get_logger().warn(
                f"[{self.cf_name}] FOLLOW mission cancelled"
            )
            return True
        return False
