# RAT-LAB PID Controller (ROS 2 Gazebo Drone Simulation)

This repository provides a ROS 2-based simulation environment for a drone running in **Gazebo**, along with nodes for pose listening and command publishing.

It uses ROS 2 for communication between nodes and Gazebo for physics-based drone simulation.

---

## 📌 Features

-   Drone simulation in Gazebo
-   ROS 2-based architecture
-   Launch file to spawn and control drone in simulation
-   Pose listener node for tracking drone state
-   Command publisher node for sending control inputs
-   Modular setup for testing PID control systems

---

## ⚙️ Prerequisites

Make sure you have the following installed:

-   ROS 2 (Jazzy recommended)
-   Gazebo (compatible version with ROS 2 Jazzy)
-   `colcon` build tools
-   `git`

Source ROS 2 before running any commands:

bash

Copy

```bash
source /opt/ros/jazzy/setup.bash
```

---

## 📁 Workspace Setup

Create a ROS 2 workspace and clone the repository:

bash

Copy

```bash
mkdir -p rat-lab-ws/srccd rat-lab-ws/srcgit clone -b pid-controller https://github.com/UPB-RAT/nonlinear_control_course_ss26.gitshopt -s dotglob && mv nonlinear_control_course_ss26/* . && rmdir nonlinear_control_course_ss26
```

---

## 🔨 Build Instructions

From the workspace root:

bash

Copy

```bash
cd ../..colcon build
```

After building, source the workspace:

bash

Copy

```bash
source install/setup.bash
```

---

## 🚀 Running the Simulation with Twist based control

We have provided a working Quadcopter PID controller using Twist message for position control. Simulation to run this version

### 1\. Launch Drone Simulation (Gazebo)

Open a new terminal:

bash

Copy

```bash
source /opt/ros/jazzy/setup.bashsource install/setup.bashros2 launch drone_simulation twist.sim.launch.py
```

This will:

-   Start Gazebo
-   Spawn the drone model
-   Initialize simulation environment

---

### 2\. Commands to run the Twist PID controller

Open a new terminal:

bash

Copy

```bash
source /opt/ros/jazzy/setup.bashsource install/setup.bashros2 launch drone_pid_controller drone_twist.launch.py 
```

---

### 3\. Commands to run the Twist PID controller with minimum snap

Open another terminal:

bash

Copy

```bash
source /opt/ros/jazzy/setup.bashsource install/setup.bashros2 launch drone_pid_controller drone_twist_snap.launch.py 
```

## 🚀 Instructions for Homework 1, Question 3 submission

We have already provided you with boilerplate code for the PID control in the file "drone\_pid\_controller/drone\_controller.py". Students are required to add the required code in the respective TODO sections and make sure the full code works successfully. You can make changes at other sections of the code if required.

### 1\. Launch Drone Simulation (Gazebo)

Open a new terminal:

bash

Copy

```bash
source /opt/ros/jazzy/setup.bashsource install/setup.bashros2 launch drone_simulation sim.launch.py
```

This will:

-   Start Gazebo
-   Spawn the drone model
-   Initialize simulation environment

---

### 2\. Commands to run the PID controller

Open a new terminal:

bash

Copy

```bash
source /opt/ros/jazzy/setup.bashsource install/setup.bashros2 launch drone_pid_controller drone.launch.py 
```

---