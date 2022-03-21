## Installation

    cd <your_ws>
    rosdep install --from-paths src -i -r -y
    colcon build
    source install/setup.bash

## Running the relay:

    ros2 run joint_trajectory_relay joint_trajectory_relay.py

- Subscribed Topic: `/robot_arm_controller/state` ([control_msgs/msg/JointTrajectoryControllerState](https://github.com/ros-controls/control_msgs/blob/galactic-devel/control_msgs/msg/JointTrajectoryControllerState.msg))

- Published Topic: `/position_trajectory_controller/joint_trajectory` ([trajectory_msgs/msg/JointTrajectory](https://docs.ros2.org/galactic/api/trajectory_msgs/msg/JointTrajectory.html))