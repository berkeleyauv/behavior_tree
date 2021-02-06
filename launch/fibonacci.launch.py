from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='behavior_tree',
            namespace='',
            executable='bt_engine',
            # Do not declare a node name otherwise it messes with the action node names and will result in
            # duplicate nodes!
            parameters=[
                {'bt_file_path': os.path.expanduser('~/Github/robosub/src/behavior_tree/trees/demo.xml')},
                {'plugins': ['fibonacci_bt_node']}
            ]
        ),
    ])
