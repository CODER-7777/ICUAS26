import rclpy
from rclpy.node import Node
from enum import IntEnum
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32

# Constants
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

class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self.fleet = [Drone(i+1) for i in range(5)]

        # Role publishers
        self.role_pubs = {d.id: self.create_publisher(Int32, f'/cf_{d.id}/role', 10) for d in self.fleet}

        # Battery subscribers
        for drone in self.fleet:
            self.create_subscription(
                BatteryState,
                f'/cf_{drone.id}/battery_status',
                lambda msg, d=drone: self.battery_callback(msg, d),
                10
            )

        self.assign_initial_roles()

        # Timers
        self.create_timer(1.0, self.manage_fleet)
        self.create_timer(1.0, self.publish_all_roles)
        # NEW: Timer to print battery status
        self.create_timer(2.0, self.print_fleet_status)

        self.get_logger().info('Fleet Manager started with Battery Monitoring')

    def battery_callback(self, msg: BatteryState, drone: Drone):
        # Multiply by 100 if the incoming message is 0.0 - 1.0
        drone.batt_percentage = msg.percentage

    def print_fleet_status(self):
        """Prints the current status of every drone in the fleet."""
        self.get_logger().info("-" * 30)
        for d in self.fleet:
            self.get_logger().info(f"Drone {d.id} | Role: {d.role.name:8} | Battery: {d.batt_percentage:.1f}%")
        self.get_logger().info("-" * 30)

    def manage_fleet(self):
        center = self.fleet[0]
        follow = self.fleet[1]

        # Send low-battery scouts to charge
        for drone in self.fleet[2:]:
            if drone.role == Role.SCOUT and drone.batt_percentage < BATTERY_PERCENTAGE_THRESH:
                drone.role = Role.CHARGE
                self.get_logger().info(f'Drone {drone.id} (Scout) sent to CHARGE')

        # Replace CENTER / FOLLOW if critical
        if center.batt_percentage < CRITICAL_BATTERY_THRESH:
            self.replace_active(center, Role.CENTER)

        if follow.batt_percentage < CRITICAL_BATTERY_THRESH:
            self.replace_active(follow, Role.FOLLOW)

    def replace_active(self, failing_drone: Drone, role: Role):
        """Finds the healthiest available drone to take over an active role."""
        # Find candidates: Any drone that isn't the one failing and has >60% battery
        candidates = [d for d in self.fleet if d.id != failing_drone.id and d.batt_percentage > BATTERY_PERCENTAGE_THRESH]

        if not candidates:
            self.get_logger().warn(f'CRITICAL: No healthy drones available to replace {role.name}')
            return

        # Selection: Pick the drone with the highest battery
        replacement = max(candidates, key=lambda d: d.batt_percentage)

        self.get_logger().info(f'SWAP: Drone {replacement.id} replaces {failing_drone.id} as {role.name}')
        
        failing_drone.role = Role.CHARGE
        replacement.role = role

    def publish_all_roles(self):
        for drone in self.fleet:
            msg = Int32()
            msg.data = int(drone.role)
            self.role_pubs[drone.id].publish(msg)

    def assign_initial_roles(self):
        self.fleet[0].role = Role.CENTER
        self.fleet[1].role = Role.FOLLOW
        for i in range(2, 5):
            self.fleet[i].role = Role.SCOUT

def main():
    rclpy.init()
    node = FleetManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()