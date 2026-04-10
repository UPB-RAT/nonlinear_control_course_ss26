import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='bebop1',
        description='Robot name'
    )
    robot_name = LaunchConfiguration('robot_name')

    pkg_controller = get_package_share_directory('controller')

    drone_controller = Node(
        package='controller',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(pkg_controller, 'config', 'waypoints.yaml'),
            {'robot_name': robot_name},
        ],
    )

    return LaunchDescription([
        robot_name_arg,
        drone_controller,
    ])