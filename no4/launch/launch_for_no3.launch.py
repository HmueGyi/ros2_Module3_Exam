from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='no3',
            executable='nodeA',
            name='node_A'
        ),
        Node(
            package='no3',
            executable='nodeB',
            name='node_B'
        )
    ])
