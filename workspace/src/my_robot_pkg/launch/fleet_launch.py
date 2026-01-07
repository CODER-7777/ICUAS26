from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1. Start the Fleet Manager (The "Brain")
    manager_node = Node(
        package='my_robot_pkg',
        executable='fleet_manager',
        name='fleet_manager'
    )
    ld.add_action(manager_node)

    # 2. Start 5 Battery Nodes (The "Drones")
    for i in range(5):
        drone_node = Node(
            package='my_robot_pkg',
            executable='battery_node',
            name=f'battery_node_{i}',
            parameters=[{'drone_id': i}]
        )
        ld.add_action(drone_node)

    return ld