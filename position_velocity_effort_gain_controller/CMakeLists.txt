cmake_minimum_required(VERSION 3.5)
project(position_velocity_effort_gain_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#1. find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(controller_interface REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_hardware_interface_bolt REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(realtime_tools REQUIRED)
  find_package(std_msgs REQUIRED)


#2. compile directive for a shared library providing the cpp file as the source
add_library(${PROJECT_NAME}
  SHARED
  src/position_velocity_effort_gain_controller.cpp
)

#3.
target_include_directories(${PROJECT_NAME} PRIVATE include)

#4. Dependencies
ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  controller_interface
  hardware_interface
  ros2_hardware_interface_bolt
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  std_msgs
)

#5. pluginlib description
target_compile_definitions(${PROJECT_NAME} PRIVATE "POSITION_VELOCITY_EFFORT_GAIN_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(${PROJECT_NAME} position_velocity_effort_gain_controller_plugins.xml)

#6. install directives for targets and include directory.
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

#9. ament export libraries
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()