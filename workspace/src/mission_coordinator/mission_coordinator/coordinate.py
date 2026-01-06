import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from enum import IntEnum
import re

class Role(IntEnum):
    """Enumeration for Drone Roles"""
    UNASSIGNED = 0
    CHARGE = 1
    SCOUT = 2
    LAND = 3
    FOLLOW = 4
    CENTER = 5

class DroneRoleManager(Node):
    def __init__(self):
        super().__init__('drone_role_manager')
        
        # Publisher using Int32MultiArray to send a list of numeric roles
        self.publisher_ = self.create_publisher(Int32MultiArray, 'drone_fleet_roles', 10)
        
        # Timer to update roles (0.5 Hz)
        self.timer = self.create_timer(2.0, self.update_roles)
        
        self.get_logger().info("Drone Role Manager started using Numeric Enums.")


    def update_roles(self):
        drone_ids = list(range(1,5+1))

        for i, drone_id in enumerate(drone_ids):
            # Collect Data for each drone
            pass

        # Prepare the Int32MultiArray message
        msg = Int32MultiArray()
        role_list = []

        for i, drone_id in enumerate(drone_ids):
            # Assigning roles based on the index
            if i == 0:
                role = Role.CENTER
            elif i % 2 == 0:
                role = Role.SCOUT
            else:
                role = Role.CHARGE
            
            # We append the role value (int) to the list
            role_list.append(int(role))
            self.get_logger().info(f"Drone {drone_id} assigned Role {int(role)} ({role.name})")
        msg.data = role_list
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneRoleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
