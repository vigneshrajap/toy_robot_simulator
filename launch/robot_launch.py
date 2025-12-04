from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='toy_robot_simulator', executable='robot_node', name='robot_controller')
    ])
