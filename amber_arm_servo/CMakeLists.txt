cmake_minimum_required(VERSION 3.5)
project(amber_arm_servo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  moveit_msgs
  geometry_msgs
)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  moveit_msgs
  geometry_msgs
)
install(
  TARGETS
    teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
