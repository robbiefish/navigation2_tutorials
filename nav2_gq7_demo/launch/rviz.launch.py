import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    nav2_gq7_dir = get_package_share_directory("nav2_gq7_demo")
    display_file = os.path.join(nav2_gq7_dir, 'rviz', 'navigation.rviz')

    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', display_file],
            output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # RViz launch
    ld.add_action(rviz_cmd)

    return ld
