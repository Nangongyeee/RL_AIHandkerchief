from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piper_rl_deploy',
            executable='coordinate_test',
            name='coordinate_test_node',
            output='screen'
        )
    ])