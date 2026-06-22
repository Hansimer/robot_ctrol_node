from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_ctrol_node',
            executable='robot_ctrol_node',
            name='robot_ctrol_node',
            output='screen',
            emulate_tty=True,
        ),
    ])