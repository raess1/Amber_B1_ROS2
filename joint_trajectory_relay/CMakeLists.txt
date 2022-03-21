cmake_minimum_required(VERSION 3.5)
project(joint_trajectory_relay)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
ament_python_install_package(scripts)

install(PROGRAMS
  scripts/joint_trajectory_relay.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()