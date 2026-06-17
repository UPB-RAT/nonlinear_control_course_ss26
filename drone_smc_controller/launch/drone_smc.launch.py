from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("drone_smc_controller")

    rviz_config = os.path.join(pkg_share, "config", "drone_view.rviz")
    smc_params = os.path.join(pkg_share, "config", "smc_params.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",   # set true if your RViz environment is fixed
        description="Launch RViz2"
    )

    smc_node = Node(
        package="drone_smc_controller",
        executable="smc_controller_node",
        name="quadcopter_smc",
        output="screen",
        parameters=[smc_params],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        use_rviz_arg,
        smc_node,
        rviz_node,
    ])