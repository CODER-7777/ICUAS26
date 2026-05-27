from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",
            "tls": False,
            "use_sim_time": False,
        }],
    )

    viz = Node(
        package="swarm_viz",
        executable="viz_node",
        name="swarm_viz",
        output="screen",
    )

    return LaunchDescription([bridge, viz])
