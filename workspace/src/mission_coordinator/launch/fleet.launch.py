from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1. Start the Fleet Manager Node
    manager_node = Node(
        package='mission_coordinator',
        executable='manager_node',
        output='screen'
    )
    ld.add_action(manager_node)

    # 2. Start Battery Nodes for 5 drones (cf_0 to cf_4)
    for i in range(5):
        battery_node = Node(
            package='mission_coordinator',
            executable='battery_node',
            name=f'battery_node_cf_{i}',
            parameters=[{'drone_id': i}],
            output='screen'
        )
        ld.add_action(battery_node)

    return ld
