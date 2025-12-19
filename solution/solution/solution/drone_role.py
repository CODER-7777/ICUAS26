#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from solution_interfaces.srv import Assign


class DroneRolesPublisher(Node):
    ALLOWED_ROLES = {'center', 'tagger', 'charging'}

    def __init__(self):
        super().__init__('drone_roles_publisher')
        default_drones = ['cf_1', 'cf_2', 'cf_3', 'cf_4', 'cf_5']
        self.drones = default_drones

        self.declare_parameter('center', '')
        self.declare_parameter('tagger', '')
        self.declare_parameter('charging', '')
        self.declare_parameter('publish_rate', 1.0)  # Hz

        self.srv = self.create_service(Assign, 'assign_roles', self.assign_roles_callback)
        self.pub = self.create_publisher(String, '/drone_roles', 10)

        rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(f'Publishing drone roles on topic: "/drone_roles" at {rate} Hz')

    def assign_roles_callback(self, request, response):
        drone_name = request.drone_name.strip()
        role = request.role.strip().lower()

        if role not in self.ALLOWED_ROLES:
            self.get_logger().warn(f'Invalid role "{request.role}" requested.')
            response.success = False
            return response

        if drone_name not in self.drones:
            self.get_logger().warn(f'Invalid drone name "{drone_name}" requested.')
            response.success = False
            return response

        self.set_parameters([rclpy.parameter.Parameter(role, rclpy.Parameter.Type.STRING, drone_name)])
        self.get_logger().info(f'Assigned role "{role}" to drone "{drone_name}".')
        response.success = True
        return response

    def timer_callback(self):
        drone_names = self.drones

        def get_role_param(role):
            return self.get_parameter(role).get_parameter_value().string_value or ''

        center = get_role_param('center')
        tagger = get_role_param('tagger')
        charging = get_role_param('charging')
        search_squad = [dn for dn in drone_names if dn not in {center, tagger, charging}]

        roles = {
            'center': [center] if center else [],
            'tagger': [tagger] if tagger else [],
            'charging': [charging] if charging else [],
            'search_squad': search_squad
        }

        msg = String()
        msg.data = json.dumps(roles)
        self.pub.publish(msg)
        self.get_logger().debug(f'Published roles: {msg.data}')


def main(args=None):
    # initialize the rclpy library
    rclpy.init(args=args)

    # create the node
    node = DroneRolesPublisher()

    # spin the node so the callback function is called.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
