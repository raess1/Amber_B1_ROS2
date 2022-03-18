cmake_minimum_required(VERSION 3.5)
project(amber_b1_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY config launch amber/meshes rviz urdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
