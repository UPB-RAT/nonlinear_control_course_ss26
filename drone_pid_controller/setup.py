from setuptools import find_packages, setup

package_name = 'drone_pid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drone.launch.py']),
        ('share/' + package_name + '/launch', ['launch/drone_twist.launch.py']),
        ('share/' + package_name + '/launch', ['launch/drone_twist_snap.launch.py']),
        ('share/' + package_name + '/launch', ['launch/fl_drone.launch.py']),
        ('share/' + package_name + '/config', ['config/drone_view.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kasm-user',
    maintainer_email='kasm-user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = drone_pid_controller.drone_controller:main',
            'twist_controller_node = drone_pid_controller.drone_twist_controller:main',
            'snap_controller_node = drone_pid_controller.drone_twist_controller_with_min_snap:main',
            'fl_controller_node = drone_pid_controller.drone_fl_controller:main'
        ],
    },
)
