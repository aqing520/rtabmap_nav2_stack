import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(get_package_share_directory("bt468_rtk_driver"), "config", "bt468_rtk.yaml")
    return LaunchDescription(
        [
            Node(
                package="bt468_rtk_driver",
                executable="bt468_rtk_node",
                name="bt468_rtk_node",
                output="screen",
                parameters=[config],
            )
        ]
    )
