from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Spawn turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim1'
        ),

        # Broadcast turtle1
        Node(
            package='no7',
            executable='broadcaster',
            name='broadcaster1',
            parameters=[{'turtlename': 'turtle1'}]
        ),

        # Broadcast turtle2
        Node(
            package='no7',
            executable='broadcaster',
            name='broadcaster2',
            parameters=[{'turtlename': 'turtle2'}]
        ),

        # Turtle2 follows turtle1
        Node(
            package='no7',
            executable='spawn_turtles',
            name='turtle2_listener',
            parameters=[
                {"target_frame": "turtle1"},
                {"turtle_name": "turtle2"},
                {"spawn_x": 4.0},
                {"spawn_y": 2.0},
                {"spawn_theta": 0.0},
            ],
        ),

    ])
