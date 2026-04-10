# NLC Drone Simulation & Controller

ROS2 Humble + Gazebo Ignition simulation of Parrot Bebop 2 drones with PID position control.

---

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Ignition (Fortress or Garden)
- Python 3.10+

```bash
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
```

---

## Codebase Structure
```
nlc_ws/
└── src/
    ├── bebop_gz/                        # Gazebo simulation package
    │   ├── models/
    │   │   └── parrot_bebop_2/          # Bebop 2 drone model
    │   │       ├── meshes/              # 3D mesh files
    │   │       ├── model.config
    │   │       └── model.sdf
    │   ├── plugins/                     # Custom Gazebo plugins
    │   │   ├── build/
    │   │   │   ├── libRobotPosePublisher.so
    │   │   │   └── libSetPosePlugin.so
    │   │   ├── RobotPosePublisher.cc
    │   │   ├── RobotPosePublisher.hh
    │   │   ├── SetPosePlugin.cc
    │   │   ├── SetPosePlugin.hh
    │   │   └── CMakeLists.txt
    │   ├── worlds/
    │   │   └── bebop_multi.world        # Multi-drone world file
    │   └── world_generator.py           # Dynamic world generator
    ├── sim_env/                         # Simulation environment package
    │   ├── config/
    │   │   └── bebop_bridge.yaml        # Bridge configuration
    │   ├── launch/
    │   │   └── sim_env.launch.py
    │   └── sim_env/
    │       └── pose_odometry.py
    └── controller/                      # Drone controller package
        ├── launch/
        │   └── controller.launch.py
        └── controller/
            ├── drone_controller.py
            ├── pid.py
            └── waypoint_manager.py
```
---

## Build

```bash
cd ~/nlc_ws
colcon build
source install/setup.bash
```

To rebuild only specific packages:
```bash
colcon build --packages-select sim_env controller
source install/setup.bash
```

---

## Running the Simulation

### Step 1 — Launch simulation environment

```bash
ros2 launch sim_env sim_env.launch.py
```

This will:
1. Generate the Gazebo world (`world_generator.py`) with 1 drone
2. Launch Gazebo Ignition with the world file
3. Start the ROS-GZ bridge (topics bridged after 12s)
4. Start the pose odometry node (after 14s)

To change the number of drones, edit `world_generator.py`:
```python
num_drones = 1   # default
```
Or pass as argument:
```bash
python3 src/bebop_gz/world_generator.py num_drones=3
```

### Step 2 — Launch the PID controller

In a new terminal:
```bash
source ~/nlc_ws/install/setup.bash
ros2 launch controller controller.launch.py
```

The drone will:
1. Arm automatically after 2 seconds
2. Fly to the default hover point `(x=0, y=0, z=1.0)`
3. Hold position until a new goal is received

---

## Sending Goals at Runtime

### Simple point (x, y, z) — yaw unchanged

```bash
ros2 topic pub --once /bebop1/goal geometry_msgs/msg/Point \
  "{x: 2.0, y: 1.0, z: 1.5}"
```

### Full pose (x, y, z + yaw)

```bash
ros2 topic pub --once /bebop1/goal_pose geometry_msgs/msg/Pose \
  "{position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}"
```

### Return to origin

```bash
ros2 topic pub --once /bebop1/goal geometry_msgs/msg/Point \
  "{x: 0.0, y: 0.0, z: 1.0}"
```

---

## Monitoring

```bash
# Watch drone position
ros2 topic echo /bebop1/pose

# Watch velocity commands
ros2 topic echo /bebop1/cmd_vel

# List all active topics
ros2 topic list
```

---

## PID Tuning

Default gains in `drone_controller.py`:

| Axis | Kp  | Ki   | Kd  | Max output |
|------|-----|------|-----|------------|
| X    | 0.5 | 0.08 | 0.4 | 0.5 m/s    |
| Y    | 0.5 | 0.08 | 0.4 | 0.5 m/s    |
| Z    | 0.2 | 0.02 | 0.1 | 0.08 m/s   |
| Yaw  | 1.0 | 0.01 | 0.1 | 1.0 rad/s  |

Override via launch arguments:
```bash
ros2 launch controller controller.launch.py \
  pid_z.kp:=0.3 pid_z.ki:=0.03 takeoff_height:=1.5
```

---

## Gazebo Physics Parameters

Key parameters in `world_generator.py`:

| Parameter | Value | Description |
|---|---|---|
| `maxRotVelocity` | 400.0 rad/s | Max propeller speed |
| `motorConstant` | 8.54858e-06 | Motor thrust coefficient |
| `velocityGain` | 0.5 0.5 0.5 | Velocity controller gain |
| `attitudeGain` | 0.3 0.5 0.05 | Attitude controller gain |
| `angularRateGain` | 0.1 0.1 0.05 | Angular rate gain |
| `maximumLinearAcceleration` | 0.5 0.5 0.5 | Max linear acceleration |

---

## Troubleshooting

| Problem | Solution |
|---|---|
| `ros2 topic list` fails with `rclpy.ok()` error | Run `ros2 daemon stop && ros2 daemon start` |
| Drone flies away on takeoff | Check `world_generator.py` gains — regenerate world |
| Bridge not forwarding `cmd_vel` | Set `lazy: false` for all `ROS_TO_GZ` topics in `bebop_bridge.yaml` |
| x-axis drift at hover | Increase `pid_x.ki` or reduce `attitudeGain` x component |
| Gazebo crashes on startup | Remove `-z 1000000` flag from `gz_args` in `sim_env.launch.py` |
| World file not updating | Edit `world_generator.py` directly — it regenerates the world on every launch |