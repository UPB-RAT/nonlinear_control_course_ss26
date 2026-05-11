## Running the trajectory: figure-8 following PID-controlled drone

```bash
colcon build

# Terminal 1
source install/setup.bash
ros2 launch drone_simulation sim.launch.py

# Terminal 2
source install/setup.bash
ros2 launch drone_pid_controller drone.launch.py

# Terminal 3
ros2 param set /quadcopter_pid trajectory circle
ros2 param set /quadcopter_pid trajectory spiral
ros2 param set /quadcopter_pid trajectory square
```