from sys import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, text_substitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('restful_fleet_server'),
        'server_node_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='restful_fleet_server',
            executable='restful_fleet_server',
            name='restful_fleet_server',
            output="screen",
            parameters=[params]
        ),
    ])