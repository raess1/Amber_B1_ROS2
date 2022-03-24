# Amber_B1_ROS2 Servo
## Note this is not installed or ran on the Amber b1 computer.

This package contains configuration for Amber B1 that enables its manipulation with MoveIt 2.

## Overview

Below is an overview of the included packages, with a small description of their purpose. For more information, please see README.md of each individual package.

- [**amber_b1_description**](./amber_b1_description) – URDF description of the robot
- [**amber_arm_moveit_config**](./amber_arm_moveit_config) – MoveIt 2 configuration for the robot

## Instructions

### Requirements

- **OS:** Ubuntu 20.04 (Focal)
  - Other distributions might work, but they were not tested.


### Building

Clone this repository. Then install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Create workspace for the project (can be skippid)
mkdir -p amber_ws/src && cd amber_ws
# Clone this repository
git clone https://github.com/raess1/Amber_B1_ROS2.git
# Install external dependencies via rosdep
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
# Build with colcon
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace overlay.

```bash
source amber_ws/install/local_setup.bash
```

This enables:

- Execution of scripts and examples via `ros2 run amber_* <executable>`
- Launching of setup scripts via `ros2 launch amber_* <launch_script>`
- Discoverability of shared resources

WORKING: (WP)
- `ros2 launch amber_b1_description view_robot_ex.launch.py`
- `ros2 launch amber_arm_moveit_config demo.launch.py`

TODO:
- Get the URDF work correctly from `ros2 launch amber_b1_description view_robot.launch.py`
- Add Gripper to urdf.
- Add Gripper Ctrl.
