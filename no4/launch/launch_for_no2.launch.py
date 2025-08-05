from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='no2',
            executable='pub',
            name='publisher',
            output='screen',
        ),
        
        Node(
            package='no2',
            executable='sub',
            name='subscriber',
            output='screen',
        ),

    ])
