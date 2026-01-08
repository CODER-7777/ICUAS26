import rclpy
from rclpy.node import Node
from enum import IntEnum
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32


BATTERY_PERCENTAGE_THRESH = 60.0
CRITICAL_BATTERY_THRESH = 30.0


class Role(IntEnum):
    UNASSIGNED = 0
    CHARGE = 1
    SCOUT = 2
    LAND = 3
    FOLLOW = 4
    CENTER = 5


class Drone:
    def __init__(self, drone_id):
        self.id = drone_id
        self.batt_percentage = 100.0
        self.role = Role.UNASSIGNED
        self.sub = None


class FleetManager(Node):

    def __init__(self):
        super().__init__('fleet_manager')

        # -------------------------
        # Create fleet
        # -------------------------
        self.fleet = [Drone(i) for i in range(5)]

        # -------------------------
        # Role publishers
        # -------------------------
        self.role_pubs = {}
        for drone in self.fleet:
            self.role_pubs[drone.id] = self.create_publisher(
                Int32,
                f'/cf_{drone.id}/role',
                10
            )

        # -------------------------
        # Battery subscribers
        # -------------------------
        for drone in self.fleet:
            self.create_subscription(
                BatteryState,
                f'/cf_{drone.id}/battery_status',
                lambda msg, d=drone: self.battery_callback(msg, d),
                10
            )

        # -------------------------
        # Assign initial roles
        # -------------------------
        self.assign_initial_roles()

        # -------------------------
        # Timers
        # -------------------------
        self.create_timer(1.0, self.manage_fleet)
        self.create_timer(1.0, self.publish_all_roles)  # CRITICAL FIX

        self.get_logger().info('Fleet Manager started')

    # -------------------------
    # Role publishing helpers
    # -------------------------
    def publish_role(self, drone: Drone):
        msg = Int32()
        msg.data = int(drone.role)
        self.role_pubs[drone.id].publish(msg)

    def publish_all_roles(self):
        for drone in self.fleet:
            self.publish_role(drone)

    # -------------------------
    # Initial role assignment
    # -------------------------
    def assign_initial_roles(self):
        self.fleet[0].role = Role.CENTER
        self.fleet[1].role = Role.FOLLOW

        for i in range(2, 5):
            self.fleet[i].role = Role.SCOUT

    # -------------------------
    # Battery callback
    # -------------------------
    def battery_callback(self, msg: BatteryState, drone: Drone):
        drone.batt_percentage = msg.percentage

    # -------------------------
    # Fleet logic
    # -------------------------
    def manage_fleet(self):
        center = self.fleet[0]
        follow = self.fleet[1]

        # Send low-battery scouts to charge
        for drone in self.fleet[2:]:
            if drone.role == Role.SCOUT and drone.batt_percentage < BATTERY_PERCENTAGE_THRESH:
                drone.role = Role.CHARGE
                self.get_logger().info(
                    f'Drone {drone.id} sent to CHARGE')

        # Replace CENTER / FOLLOW if critical
        if center.batt_percentage < CRITICAL_BATTERY_THRESH:
            self.replace_active(center, Role.CENTER)

        if follow.batt_percentage < CRITICAL_BATTERY_THRESH:
            self.replace_active(follow, Role.FOLLOW)

    # -------------------------
    # Replacement logic
    # -------------------------
    def replace_active(self, failing_drone: Drone, role: Role):
        charging = [
            d for d in self.fleet
            if d.role == Role.CHARGE and d.batt_percentage > BATTERY_PERCENTAGE_THRESH
        ]

        if not charging:
            self.get_logger().warn(
                f'No charged drones available to replace {role.name}')
            return

        replacement = max(charging, key=lambda d: d.batt_percentage)

        self.get_logger().info(
            f'Replacing Drone {failing_drone.id} with Drone {replacement.id}')

        failing_drone.role = Role.LAND
        replacement.role = role

        if replacement.batt_percentage < CRITICAL_BATTERY_THRESH:
            self.get_logger().warn(
                f'Drone {replacement.id} also critical, landing it')

            replacement.role = Role.LAND

            if failing_drone.batt_percentage > BATTERY_PERCENTAGE_THRESH:
                failing_drone.role = role
                self.get_logger().info(
                    f'Drone {failing_drone.id} restored to {role.name}')


def main():
    rclpy.init()
    node = FleetManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()