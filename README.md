# RAT-LAB PID Controller (ROS 2 Gazebo Drone Simulation)

This repository provides a ROS 2-based simulation environment for a drone running in **Gazebo**, along with nodes for pose listening and command publishing.

It uses ROS 2 for communication between nodes and Gazebo for physics-based drone simulation.

---

## 📌 Features

* Drone simulation in Gazebo
* ROS 2-based architecture
* Launch file to spawn and control drone in simulation
* Pose listener node for tracking drone state
* Command publisher node for sending control inputs
* Modular setup for testing PID control systems

---

## ⚙️ Prerequisites

Make sure you have the following installed:

* ROS 2 (Jazzy recommended)
* Gazebo (compatible version with ROS 2 Jazzy)
* `colcon` build tools
* `git`

Source ROS 2 before running any commands:

```bash
source /opt/ros/jazzy/setup.bash
```

---

## 📁 Workspace Setup

Create a ROS 2 workspace and clone the repository:

```bash
mkdir -p rat-lab-ws/src
cd rat-lab-ws/src

git clone https://github.com/ekansh-bajpai/rat-lab-pid-controller.git .
```

---

## 🔨 Build Instructions

From the workspace root:

```bash
cd ../..
colcon build
```

After building, source the workspace:

```bash
source install/setup.bash
```

---

## 🚀 Running the Simulation

### 1. Launch Drone Simulation (Gazebo)

In the first terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch drone_simulation sim.launch.py
```

This will:

* Start Gazebo
* Spawn the drone model
* Initialize simulation environment

---

### 2. Start Pose Listener Node

Open a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run drone_communication listener_node
```

This node:

* Subscribes to drone state/pose topics
* Processes real-time position and orientation data

---

### 3. Start Command Publisher Node

Open another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run drone_communication publisher_node
```

This node:

* Publishes control commands to the drone
* Can be extended for PID tuning and trajectory control
