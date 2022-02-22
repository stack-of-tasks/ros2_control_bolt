cmake_minimum_required(VERSION 3.5)
project(ros2_description_bolt)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes ros2_control urdf launch gazebo config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
