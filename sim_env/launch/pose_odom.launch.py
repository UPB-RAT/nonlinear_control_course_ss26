import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    IncludeLaunchDescription, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Arguments ──────────────────────────────────────────────────────────────
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='bebop1',
        description='Robot name'
    )
    robot_name = LaunchConfiguration('robot_name')

    # ── Package paths ───────────────────────────────────────────────────────────
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_sim_env    = get_package_share_directory('sim_env')

    # ── Gazebo environment paths ────────────────────────────────────────────────
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.expanduser('~/nlc_ws/src/bebop_gz/models')
    )

    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.expanduser('~/nlc_ws/install/GazeboPlugins/lib')
    )

    # ── 1. World generator (runs immediately) ───────────────────────────────────
    world_generator = ExecuteProcess(
        cmd=[
            'python3',
            os.path.expanduser('~/nlc_ws/src/bebop_gz/world_generator.py'),
            'num_drones=1'
        ],
        output='screen'
    )

    # ── 2. Gazebo simulator ─────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r ' + os.path.expanduser(
                '~/nlc_ws/src/bebop_gz/worlds/bebop_multi.world'
            ),
        }.items(),
    )

    # ── 3. ROS-GZ bridge ────────────────────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_sim_env, 'config', 'bebop_bridge.yaml'),
        }],
    )

    # ── 4. Pose odometry ────────────────────────────────────────────────────────
    pose_odometry = Node(
        package='sim_env',
        executable='pose_odometry',
        name='pose_odometry',
        output='screen',
        emulate_tty=True,
        parameters=[{'robot_name': robot_name}],
    )

    # ── Launch sequence ─────────────────────────────────────────────────────────
    return LaunchDescription([
        gz_resource_path,
        gz_plugin_path,
        robot_name_arg,
        world_generator,
        TimerAction(
            period=5.0,
            actions=[gz_sim]
        ),
        TimerAction(
            period=12.0,
            actions=[ros_gz_bridge]
        ),
        TimerAction(
            period=14.0,
            actions=[pose_odometry]
        ),
    ])