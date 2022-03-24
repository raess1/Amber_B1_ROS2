# Amber_B1_ROS2 Servo
## Note this is not installed or ran on the Amber b1 computer.


This package contains configuration for Amber B1 that enables its manipulation with MoveIt 2 servo and a joystick.

![galactic](https://user-images.githubusercontent.com/6362413/159165112-46f7a940-4231-4605-b35d-27c4d9becec1.PNG)

## Overview

Below is an overview of the included packages, with a small description of their purpose. For more information, please see README.md of each individual package.

- [**amber_arm_moveit_config**](./amber_arm_moveit_config) – MoveIt 2 configuration for the robot
- [**amber_arm_servo**](./amber_arm_servo) – Servo configurations
- [**amber_b1_description**](./amber_b1_description) – URDF description of the robot
- [**joint_trajectory_relay**](./joint_trajectory_relay) – Communication between moveit2 and Amber b1

## Instructions

### Requirements

- **OS:** Ubuntu 20.04 (Focal)
- **OS:** ROS2 Galactic
  - Other distributions might work, but they were not tested.

### Install ROS and MoveIt2
```bash
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
```

```bash
https://moveit.ros.org/install-moveit2/binary/
```

### Create Workspace
```bash
source /opt/ros/galactic/setup.bash
```

```bash
mkdir -p ~/amber_ws/src
cd ~/amber_ws/src
```

```bash
. install/local_setup.bash
```

### Building

Clone this repository. Then install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Create workspace for the project (can be skippid)
mkdir -p amber_ws/src && cd amber_ws
# Clone this repository
git clone https://github.com/raess1/Amber_B1_ROS2.git
# Git checkout servo branch
cd Amber_B1_ROS2
git checkout servo
cd ..
# Install external dependencies via rosdep
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
# Install backward-ros
sudo apt install ros-galactic-backward-ros
# Build with colcon
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace overlay.

```bash
source /opt/ros/galactic/local_setup.bash
```

```bash
cd amber_ws
```

```bash
. install/local_setup.bash
```

This enables:

- Execution of scripts and examples via `ros2 run amber_* <executable>`
- Launching of setup scripts via `ros2 launch amber_* <launch_script>`
- Discoverability of shared resources

### Running MoveIt2 and Amber b1

On Robot:
- `sudo -sE`
- `/home/amber/b1.sh`

On Desktop:
- Terminal 1:

```bash
ros2 launch amber_arm_moveit_config demo.launch.py
```
- Terminal 2:

```bash
ros2 run joint_trajectory_relay joint_trajectory_relay.py
```

- Terminal 3:

```bash
ros2 launch amber_arm_servo servo.launch.py
```

### Notes

- (if error) `LC_NUMERIC=en_US.UTF-8 ros2 launch amber_arm_moveit_config pos.launch.py`


TODO:
- Add Gripper to urdf.
- Add Gripper Ctrl.



