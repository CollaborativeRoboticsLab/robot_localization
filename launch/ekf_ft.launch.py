from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
	ekf_params = os.path.join(get_package_share_directory("robot_localization"),	"params", "ekf.yaml")

	ekf_node = Node(
		package="robot_localization",
		executable="ekf_ft_node",
		name="ekf_ft_filter_node",
		output="screen",
		parameters=[ekf_params],
	)

	return LaunchDescription([ekf_node])

