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

    ebimu_node = Node(
        package='imu_pkg',
        executable='imu_pub',
        name='ebimu_publisher',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        ebimu_node
    ])