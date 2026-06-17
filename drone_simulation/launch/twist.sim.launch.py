import os
import shutil

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'drone_simulation'
    pkg_share = get_package_share_directory(pkg_name)

    models_dir = os.path.join(pkg_share, 'models')
    world_dir = os.path.join(pkg_share, 'worlds')

    # ---------------------------------------------------
    # Detect ROS distro + simulator
    # ---------------------------------------------------
    ros_distro = os.environ.get('ROS_DISTRO', '').lower()

    has_gz = shutil.which('gz') is not None
    has_ign = shutil.which('ign') is not None

    gazebo_cmd = None
    resource_env = None
    msg_prefix = None

    # Iron / Jazzy / Rolling → Gazebo (gz)
    if ros_distro in ['iron', 'jazzy', 'rolling'] and has_gz:

        gazebo_cmd = [
            'gz',
            'sim',
            '-v',
            '4',
            os.path.join(world_dir, 'drone_world_twist.sdf')
        ]

        resource_env = SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_dir
        )

        msg_prefix = 'gz.msgs'

    # Humble → Ignition
    elif ros_distro in ['humble'] and has_ign:

        gazebo_cmd = [
            'ign',
            'gazebo',
            '-v',
            '4',
            os.path.join(world_dir, 'drone_world_ign_twist.sdf')
        ]

        resource_env = SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=models_dir
        )

        msg_prefix = 'ignition.msgs'

    # Fallback if ROS_DISTRO unavailable
    elif has_gz:

        gazebo_cmd = [
            'gz',
            'sim',
            '-v',
            '4',
            os.path.join(world_dir, 'drone_world_twist.sdf')
        ]

        resource_env = SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_dir
        )

        msg_prefix = 'gz.msgs'

    elif has_ign:

        gazebo_cmd = [
            'ign',
            'gazebo',
            '-v',
            '4',
            os.path.join(world_dir, 'drone_world_ign_twist.sdf')
        ]

        resource_env = SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=models_dir
        )

        msg_prefix = 'ignition.msgs'

    else:
        raise RuntimeError(
            f"No compatible Gazebo found for ROS_DISTRO='{ros_distro}'"
        )

    gazebo = ExecuteProcess(
        cmd=gazebo_cmd,
        output='screen'
    )

    ros_bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='sim_bridge',
                output='screen',
                arguments=[
                    f'/X3/gazebo/command/motor_speed@actuator_msgs/msg/Actuators@{msg_prefix}.Actuators',
                    f'/X3/gazebo/command/twist@geometry_msgs/msg/Twist@{msg_prefix}.Twist',
                    f'/world/quadcopter/pose/info@geometry_msgs/msg/PoseArray@{msg_prefix}.Pose_V',
                ]
            )
        ]
    )

    return LaunchDescription([
        resource_env,
        gazebo,
        ros_bridge,
    ])