import rclpy
from rclpy.node import Node
import threading
from enum import IntEnum

from std_msgs.msg import Int32MultiArray

from center_role import CenterTask
from charging_role import ChargingTask
from follower_role import FollowerTask


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

        # cf_name → {role, thread, cancel_event}
        self.active_roles = {}

        self.role_sub = self.create_subscription(
            Int32MultiArray,
            '/fleet/roles',
            self.role_callback,
            10
        )

        self.get_logger().info(
            "FleetRoleManager running (multi-role, multi-thread)"
        )

    def role_callback(self, msg):
        for i, role_val in enumerate(msg.data):
            cf_name = f"cf_{i+1}"
            role = Role(role_val)

            if role == Role.UNASSIGNED:
                self._stop_role(cf_name)
                continue

            self._start_or_update_role(cf_name, role)

    # ===================== CORE =====================
    def _start_or_update_role(self, cf_name, role):
        state = self.active_roles.get(cf_name)

        # Same role already active
        if state and state["role"] == role:
            return

        # Stop old role
        if state:
            self.get_logger().warn(
                f"[{cf_name}] Role change {state['role'].name} → {role.name}"
            )
            state["cancel_event"].set()
            state["thread"].join()

        if role not in ROLE_TASK_MAP:
            self.get_logger().info(
                f"[{cf_name}] Role {role.name} not implemented"
            )
            return

        cancel_event = threading.Event()
        task_cls = ROLE_TASK_MAP[role]

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
