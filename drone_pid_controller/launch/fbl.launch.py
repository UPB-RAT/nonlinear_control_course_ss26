from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Path to RViz config inside your package
    rviz_config = os.path.join(
        get_package_share_directory('drone_pid_controller'),
        'config',
        'drone_view.rviz'
    )

    return LaunchDescription([

        # -------------------------
        # Drone controller node
        # -------------------------
        Node(
            package='drone_pid_controller',
            executable='FBL_controller_node',
            name='feedback_linearization',
            output='screen'
        ),

        # -------------------------
        # RViz2
        # -------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])