from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_obstacle_field',
                '-world', 'default',  # Name of the active PX4 world
                '-file', '/home/kaycee/PX4-Autopilot/Tools/simulation/gz/models/my_custom_obstacle/model4.sdf',
                '-x', '0', '-y', '0', '-z', '0' # Base offset coordinate
            ],
            output='screen'
        )
    ])