from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='behavior_tree',
            namespace='',
            executable='bt_engine',
            name='bt_engine',
            parameters=[
                {'bt_file_path': '/home/michael/Github/robosub/src/behavior_tree/trees/demo.xml'},
                {'plugins': ['fibonacci_bt_node']}
            ]
        ),
    ])
