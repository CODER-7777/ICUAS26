from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    show_range = DeclareLaunchArgument(
        "show_comm_range_spheres", default_value="false",
        description="Render translucent comm-range spheres around each drone")
    publish_rate = DeclareLaunchArgument(
        "publish_rate", default_value="5.0",
        description="Marker republish rate (Hz)")
    trail_length = DeclareLaunchArgument(
        "trail_length", default_value="200",
        description="Pose-trail ring buffer length per drone")

    viz = Node(
        package="swarm_viz",
        executable="viz_node",
        name="swarm_viz",
        output="screen",
        parameters=[{
            "publish_rate": LaunchConfiguration("publish_rate"),
            "trail_length": LaunchConfiguration("trail_length"),
            "show_comm_range_spheres": LaunchConfiguration("show_comm_range_spheres"),
        }],
    )

    return LaunchDescription([show_range, publish_rate, trail_length, viz])
