from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
	ukf_params = os.path.join(get_package_share_directory("robot_localization"), "params", "ukf.yaml")

	ukf_node = Node(
		package="robot_localization",
		executable="ukf_ft_node",
		name="ukf_ft_filter_node",
		output="screen",
		parameters=[ukf_params],
	)

	return LaunchDescription([ukf_node])

