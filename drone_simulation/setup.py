import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_simulation'


def package_files(data_files, directory, install_dir):
    for (path, _, filenames) in os.walk(directory):
        if filenames:
            install_path = os.path.join(install_dir, path)
            file_list = [os.path.join(path, f) for f in filenames]
            data_files.append((install_path, file_list))
    return data_files


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Launch files
data_files.append(
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
)

# World files
data_files.append(
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf'))
)

# ✅ Models (preserve folder structure!)
data_files = package_files(
    data_files,
    'models',
    os.path.join('share', package_name)
)


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ekansh',
    maintainer_email='ekansh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)