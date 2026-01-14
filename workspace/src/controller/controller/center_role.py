import math
import time
import rclpy
from crazyflie_interfaces.srv import GoTo


class CenterTask:
    def __init__(
        self,
        node,
        cf_name,
        cancel_event,
        target_altitude=5.0,
        velocity=0.1
    ):
        self.node = node
        self.cf_name = cf_name
        self.cancel_event = cancel_event
        self.target_altitude = target_altitude
        self.velocity = velocity

        self.client_goto = node.create_client(
            GoTo,
            f'/{cf_name}/go_to'
        )

        while not self.client_goto.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f"[{cf_name}] Waiting for go_to service..."
            )

    # ===================== TASK =====================
    def execute(self):
        self.node.get_logger().info(
            f"[{self.cf_name}] CENTER mission started"
        )

        # -------- WAIT FOR POSE --------
        while rclpy.ok():
            if self.cancel_event.is_set():
                return

            with self.node.pose_lock:
                pose = self.node.current_pose.get(self.cf_name)

            if pose is not None:
                break

            time.sleep(0.1)

        start_x, start_y, start_z, _ = pose

        # -------- TAKEOFF --------
        if self._cancelled():
            return

        vert_dist = abs(self.target_altitude - start_z)
        duration_up = max(vert_dist / self.velocity, 4.0)

        self._send_goto(
            start_x,
            start_y,
            self.target_altitude,
            0.0,
            duration_up
        )

        # -------- MOVE TO CENTER --------
        if self._cancelled():
            return

        horiz_dist = math.sqrt(start_x**2 + start_y**2)
        duration_move = max(horiz_dist / self.velocity, 5.0)

        self.node.get_logger().info(
            f"[{self.cf_name}] Moving to (0,0)"
        )

        self._send_goto(
            0.0,
            0.0,
            self.target_altitude,
            0.0,
            duration_move
        )

        self.node.get_logger().info(
            f"[{self.cf_name}] CENTER mission completed"
        )

    # ===================== HELPERS =====================
    def _send_goto(self, x, y, z, yaw, duration):
        if self._cancelled():
            return

        req = GoTo.Request()
        req.goal.x = float(x)
        req.goal.y = float(y)
        req.goal.z = float(z)
        req.yaw = float(yaw)
        req.duration = rclpy.duration.Duration(
            seconds=duration
        ).to_msg()
        req.relative = False

        self.client_goto.call_async(req)

        end_time = time.time() + duration
        while time.time() < end_time:
            if self._cancelled():
                return
            time.sleep(0.1)

    def _cancelled(self):
        if self.cancel_event.is_set():
            self.node.get_logger().warn(
                f"[{self.cf_name}] CENTER mission cancelled"
            )
            return True
        return False
