## Installation

    cd <your_ws>
    rosdep install --from-paths src -i -r -y
    colcon build
    source install/setup.bash


## Running the servo package: 
Run the robot ARM's driver and moveit_config package. Send the end-effector to a movable pose and free from singularity. Once done, run:

    ros2 launch amber_arm_servo servo.launch.py