import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        
        # World where its response goes to the HWInterface
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg, 'worlds', 'esp_robot_world.sdf')}.items(),
        
        # Ideal World
        # launch_arguments={'gz_args': '-r ' + os.path.join(pkg, 'worlds', 'robot_world.sdf')}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        # Does the work of the HWInterface
        # parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml')}],
        # Send the info like an esp does
        parameters=[{'config_file': os.path.join(pkg, 'config', 'esp_bridge.yaml')}],
    )

    return LaunchDescription([gz, bridge])