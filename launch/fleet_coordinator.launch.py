from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fleet_coordinator',
            executable='fleet_coordinator_node',
            name='fleet_coordinator',
            output='screen',
            emulate_tty=True,
        )
    ])
