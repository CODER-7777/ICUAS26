#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from enum import IntEnum

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped

from controller.center_role import CenterTask
from controller.follower_role import FollowerTask
from controller.charging_role import ChargingTask


# ===================== ROLES =====================
class Role(IntEnum):
    UNASSIGNED = 0
    CHARGE = 1
    SCOUT = 2
    LAND = 3
    FOLLOW = 4
    CENTER = 5


# ===================== ROLE → TASK MAP =====================
ROLE_TASK_MAP = {
    Role.CENTER: CenterTask,
    Role.CHARGE: ChargingTask,
    Role.FOLLOW: FollowerTask,
}


class FleetRoleManager(Node):
    def __init__(self):
        super().__init__('fleet_role_manager')

        # ------------------ STATE ------------------
        self.active_roles = {}            # cf_name → {role, thread, cancel_event}
        self.current_pose = {}            # cf_name → (x, y, z, yaw)
        self.pose_lock = threading.Lock() # thread safety

        # ------------------ SUBSCRIPTIONS ------------------
        self.role_sub = self.create_subscription(
            Int32MultiArray,
            '/fleet/roles',
            self.role_callback,
            10
        )

        self.pose_subs = {}
        for i in range(5):  # adjust if needed
            cf_name = f"cf_{i+1}"
            self.pose_subs[cf_name] = self.create_subscription(
                PoseStamped,
                f'/{cf_name}/pose',
                lambda msg, name=cf_name: self.pose_callback(msg, name),
                10
            )

        self.get_logger().info(
            "FleetRoleManager running (per-CF state, multi-threaded)"
        )

    # ===================== CALLBACKS =====================
    def pose_callback(self, msg: PoseStamped, cf_name: str):
        pos = msg.pose.position

        # yaw ignored for now (0.0); can compute from quaternion later
        with self.pose_lock:
            self.current_pose[cf_name] = (
                pos.x,
                pos.y,
                pos.z,
                0.0
            )

    def role_callback(self, msg: Int32MultiArray):
        for i, role_val in enumerate(msg.data):
            cf_name = f"cf_{i+1}"
            role = Role(role_val)

            if role == Role.UNASSIGNED:
                self._stop_role(cf_name)
                continue

            self._start_or_update_role(cf_name, role)

    # ===================== ROLE CONTROL =====================
    def _start_or_update_role(self, cf_name, role):
        state = self.active_roles.get(cf_name)

        # Same role already active
        if state and state["role"] == role:
            return

        # Stop previous role
        if state:
            self.get_logger().warn(
                f"[{cf_name}] Role change {state['role'].name} → {role.name}"
            )
            state["cancel_event"].set()
            state["thread"].join()

        task_cls = ROLE_TASK_MAP.get(role)
        if not task_cls:
            self.get_logger().info(
                f"[{cf_name}] Role {role.name} not implemented"
            )
            return

        cancel_event = threading.Event()

        task = task_cls(
            node=self,
            cf_name=cf_name,
            cancel_event=cancel_event
        )

        thread = threading.Thread(
            target=task.execute,
            daemon=True
        )

        self.active_roles[cf_name] = {
            "role": role,
            "thread": thread,
            "cancel_event": cancel_event
        }

        self.get_logger().info(
            f"[{cf_name}] Starting role {role.name}"
        )
        thread.start()

    def _stop_role(self, cf_name):
        state = self.active_roles.get(cf_name)
        if not state:
            return

        self.get_logger().warn(
            f"[{cf_name}] Stopping role {state['role'].name}"
        )

        state["cancel_event"].set()
        state["thread"].join()
        del self.active_roles[cf_name]


# ===================== MAIN =====================
def main():
    rclpy.init()
    node = FleetRoleManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("FleetRoleManager shutting down")
    finally:
        for state in node.active_roles.values():
            state["cancel_event"].set()
            state["thread"].join()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
