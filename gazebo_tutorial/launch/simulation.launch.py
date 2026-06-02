import os
import shutil

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    pkg_name = 'gazebo_tutorial'
    pkg_share = get_package_share_directory(pkg_name)

    world = PathJoinSubstitution([
        pkg_share,
        "worlds",
        "base.world"
    ])


    models_dir = os.path.join(pkg_share, 'models')
    print(f"Models directory: {models_dir}")
    print(os.listdir(models_dir))

    # ---------------------------------------------------
    # Detect simulator
    # ---------------------------------------------------
    use_gz = shutil.which('gz') is not None
    use_ign = shutil.which('ign') is not None

    if use_gz:
        gazebo_cmd = ['gz', 'sim', '-v', '4', world]

        resource_env = SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_dir
        )

    elif use_ign:        
        gazebo_cmd = ['ign', 'gazebo', '-v', '4', world]

        resource_env = SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=models_dir
        )

    else:
        raise RuntimeError("Neither 'gz' nor 'ign' Gazebo found")
    
    gazebo = ExecuteProcess(
        cmd=gazebo_cmd,
        output='screen'
    )

    return LaunchDescription([
        resource_env,
        gazebo
    ])