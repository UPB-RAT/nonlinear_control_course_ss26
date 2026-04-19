from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable

def generate_launch_description():

    pkg_name = 'drone_simulation'
    pkg_share = get_package_share_directory(pkg_name)

    models_dir = os.path.join(pkg_share, 'models')
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_dir
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gz", "sim", "-v", "4",
            os.path.join(pkg_share, "worlds", "drone_world.sdf")
        ],
        output="screen"
    )

    ros_gz_bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                arguments=[
                    "/X3/gazebo/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
                    "/X3/gazebo/command/twist@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/world/quadcopter/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V"
                ]
            )
        ]
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        ros_gz_bridge,
    ])