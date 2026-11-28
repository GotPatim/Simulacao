from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Movement',
            executable='esp',
            name='esp',
            output='screen'
        ),
        Node(
            package='Movement',
            executable='walk',
            name='walk',
            output='screen'
        ),
        Node(
            package='Movement',
            executable='hwinterface',
            name='hwinterface',
            output='screen'
        ),
    ])