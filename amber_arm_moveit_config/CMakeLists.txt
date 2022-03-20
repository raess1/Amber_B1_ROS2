cmake_minimum_required(VERSION 3.10.2)
project(amber_arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch srdf config DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
