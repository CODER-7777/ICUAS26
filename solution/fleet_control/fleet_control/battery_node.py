import rclpy
from rclpy.node import Node
from enum import IntEnum
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32


class Role(IntEnum):
    UNASSIGNED = 0
    CHARGE = 1
    SCOUT = 2
    LAND = 3
    FOLLOW = 4
    CENTER = 5


class BatteryPublisher(Node):

    def __init__(self):
        super().__init__('battery_publisher')

        self.declare_parameter('drone_id', 0)

        self.drone_id = self.get_parameter('drone_id').value
        self.battery = 100.0
        self.role = Role.UNASSIGNED

        self.battery_pub = self.create_publisher(
            BatteryState,
            f'/cf_{self.drone_id}/battery_status',
            10
        )

        self.role_sub = self.create_subscription(
            Int32,
            f'/cf_{self.drone_id}/role',
            self.role_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.update_battery)

        self.get_logger().info(
            f'Battery node started for cf_{self.drone_id} (UNASSIGNED)'
        )

    def role_callback(self, msg: Int32):
        self.role = Role(msg.data)

    def update_battery(self):
        if self.role == Role.CHARGE:
            self.battery += 1.0
        elif self.role == Role.LAND:
            pass
        else:
            self.battery -= 1.0

        self.battery = max(0.0, min(100.0, self.battery))

        msg = BatteryState()
        msg.percentage = self.battery / 100.0
        msg.present = True

        self.battery_pub.publish(msg)


def main():
    rclpy.init()
    node = BatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()