# drone_pid_controller

The package tracks drone pose data, computes PID-based velocity commands, and visualizes the generated trajectory in RViz2.

---

## Features

- PID control for:
  - X position
  - Y position
  - Z altitude
- Multiple trajectory generators:
  - Figure-8
  - Circle
  - Spiral
  - Square
- Runtime trajectory switching using ROS 2 parameters
- Real-time path visualization in RViz2
- Modular controller and trajectory architecture

---

## Package Structure

```text
drone_pid_controller/
├── config/
│   └── drone_view.rviz
├── drone_pid_controller/
│   ├── drone_controller.py
│   ├── pid.py
│   └── trajectory.py
├── launch/
│   └── drone.launch.py
├── package.xml
├── setup.py
└── Readme.md
```

---

## Build

From the root of your ROS 2 workspace:

```bash
colcon build --packages-select drone_pid_controller
source install/setup.bash
```

---

## Run

### Start Simulator

```bash
ros2 launch drone_simulation sim.launch.py
```

### Start Controller

```bash
ros2 launch drone_pid_controller drone.launch.py
```

---

## Runtime Trajectory Switching

Switch trajectories dynamically using ROS 2 parameters:

### Circle

```bash
ros2 param set /quadcopter_pid trajectory circle
```

### Spiral

```bash
ros2 param set /quadcopter_pid trajectory spiral
```

### Square

```bash
ros2 param set /quadcopter_pid trajectory square
```

### Figure-8

```bash
ros2 param set /quadcopter_pid trajectory figure8
```

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/X3/gazebo/command/twist` | `geometry_msgs/Twist` | Drone velocity commands |
| `/drone_path` | `nav_msgs/Path` | Visualized drone trajectory |

---

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/world/quadcopter/pose/info` | `geometry_msgs/PoseArray` | Drone pose feedback |

---

## PID Gains

| Axis | Kp | Ki | Kd |
|---|---|---|---|
| X | 1.2 | 0.0 | 0.4 |
| Y | 1.2 | 0.0 | 0.4 |
| Z | 1.8 | 0.0 | 0.5 |

---

## Launch File

```bash
ros2 launch drone_pid_controller drone.launch.py
```

Launches:

- PID controller node
- RViz2 visualization
