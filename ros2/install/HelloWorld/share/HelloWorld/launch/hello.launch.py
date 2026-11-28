from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='HelloWorld',
            executable='exec_pub',
            name='hello_pub'
        ),
        Node(
            package='HelloWorld',   
            executable='exec_sub',
            name='hello_sub'
        )
    ])

"""
    colcon build --packages-select HelloWorld -> Compila o codigo
    source install/setup.bash -> atualiza o terminal pra registrar o codigo novo
    ros2 launch HelloWorld hello.launch.py -> executa os nós
"""