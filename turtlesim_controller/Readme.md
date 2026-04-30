# Turtlesim Controller (ROS 2)

A ROS 2 Python package for controlling the `turtlesim` robot using PID control and trajectory tracking from predefined or JSON-based paths.

---

## Features

* PID-based motion control (distance + heading)
* Trajectory tracking (square, figure-8, waypoint JSON)
* Live PID tuning dashboard
* JSON-based trajectory loading via ROS parameters

---

## Creating a ROS 2 Python Package

```bash
cd <path/to/ros2_workspace>/src
ros2 pkg create --build-type ament_python turtlesim_controller
```

---

## setup.py (Important Parts)

Your package should include:

```python
from setuptools import setup

package_name = 'turtlesim_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', [
            'resource/*.json'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    description='Turtlesim PID and trajectory controller',
    license='Apache License 2.0',
    entry_points={
        ## In this part make sure to add the paths to the main function of your nodes.
        ## This enables to run the ros package with command "ros2 run <your_ros2_package> <your_ros2_node>"
        'console_scripts': [
            'pid_controller = turtlesim_controller.pid_controller_node:main',
            'trajectory_tracker = turtlesim_controller.trajectory_tracking_controller:main',
        ],
    },
)
```

---

## Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

---

## Running the System

### 1. Start turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

---

### 2. Run PID Controller (built-in trajectory)

```bash
ros2 run turtlesim_controller pid_controller
```

**What it does:**

* Uses `SquareTrajectory` (default in code)
* Controls turtle via:

  * `/turtle1/pose` (subscriber)
  * `/turtle1/cmd_vel` (publisher)

---

### 3. Run Trajectory Tracking Controller (JSON-based)

```bash
ros2 run turtlesim_controller trajectory_tracker
```

---

### Use Custom Trajectory File

```bash
ros2 run turtlesim_controller trajectory_tracker \
  --ros-args -p traj_file:=flower.json
```

Available examples:

* `flower.json`
* `star.json`
* `diamond.json`
* `butterfly.json`

---

## Topics Used

| Topic              | Type                      | Description       |
| ------------------ | ------------------------- | ----------------- |
| `/turtle1/pose`    | `turtlesim/msg/Pose`      | Robot state       |
| `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands |

---

## Parameters

### trajectory_tracking_controller

| Parameter   | Type   | Default     | Description               |
| ----------- | ------ | ----------- | ------------------------- |
| `traj_file` | string | `star.json` | JSON trajectory to follow |

---

## PID Controller Details

Two PID loops are used:

* **Distance PID**

  * Controls forward velocity
* **Heading PID**

  * Controls angular velocity

Default gains:

```text
Distance: kp=1.4, ki=0.0, kd=0.15
Heading:  kp=6.0, ki=0.0, kd=0.35
```

---

## Live PID Dashboard

* Real-time tuning of PID gains
* Reset integrators
* Enabled by default in code

---

## Example Workflow

```bash
# Build
colcon build

# Source
source install/setup.bash

# Run simulator
ros2 run turtlesim turtlesim_node

# Run trajectory tracking
ros2 run turtlesim_controller trajectory_tracker \
  --ros-args -p traj_file:=flower.json
```
