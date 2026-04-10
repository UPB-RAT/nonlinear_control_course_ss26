from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            executable='pid_controller_min_snap',
            name='pid_controller_min_snap',
            parameters=[{
                'robot_name': 'bebop1',
                'waypoints': [
                    # x       y       z     yaw        # description
                    0.000,  0.000,  0.000,  0.0000,   # spawn
                    0.000,  0.000,  2.000,  0.0000,   # takeoff to 2m
                    0.000,  0.000,  2.000,  0.7854,   # figure-8 center (start)
                    1.414,  1.000,  2.000,  0.0000,   # top-right lobe
                    2.000,  0.000,  2.000, -1.5708,   # right apex
                    1.414, -1.000,  2.000, -3.1416,   # bottom-right lobe
                    0.000,  0.000,  2.000,  2.3562,   # center crossover
                   -1.414,  1.000,  2.000,  3.1416,   # top-left lobe
                   -2.000,  0.000,  2.000, -1.5708,   # left apex
                   -1.414, -1.000,  2.000,  0.0000,   # bottom-left lobe
                    0.000,  0.000,  2.000,  0.0000,   # return to center
                ],
            }],
            output='screen',
        )
    ])