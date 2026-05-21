from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	params_file = PathJoinSubstitution([
		FindPackageShare('imu_pkg'),
		'config',
		'params.yaml'
	])
	rviz_file = PathJoinSubstitution([
		FindPackageShare('imu_pkg'),
		'config',
		'default.rviz'
	])
	ebimu_node = Node(
		package='imu_pkg',
		executable='imu_pub',
		name='ebimu_publisher',
		output='screen',
		parameters=[params_file]
	)

	rviz_node = Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			output='screen',
			arguments=['-d', rviz_file]
	)

	return LaunchDescription([
		ebimu_node,
		rviz_node
	])