from setuptools import find_packages, setup

package_name = 'turtlesim_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/traj.json']),
        ('share/' + package_name + '/resource', ['resource/butterfly.json']),
        ('share/' + package_name + '/resource', ['resource/flower.json']),
        ('share/' + package_name + '/resource', ['resource/star.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huyen-admin',
    maintainer_email='vhuyendang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_controller_node = turtlesim_controller.pid_controller_node:main',
            "trajectory_controller_node = turtlesim_controller.trajectory_tracking_controller:main"
        ],
    },
)
