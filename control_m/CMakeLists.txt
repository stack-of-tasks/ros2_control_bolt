cmake_minimum_required(VERSION 3.5)
project(control_m)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#These variables have to be defined before running SETUP_PROJECT
set(PROJECT_NAME control_m)
set(PROJECT_DESCRIPTION
    "Common interface for controlling robots build with the odri master board. Here it's for Bolt")
set(PROJECT_URL
    https://github.com/PaulRouanet/ros2_hardware_interface_bolt.git)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(PYTHON_STANDARD_LAYOUT "Enable standard Python package layout" ON)
option(PYTHON_DEB_LAYOUT "Enable Debian-style Python package layout" OFF)

# include(cmake/base.cmake)
# include(cmake/boost.cmake)
# include(cmake/python.cmake)
# include(cmake/ide.cmake)

# compute_project_args(PROJECT_ARGS LANGUAGES CXX)
# project(${PROJECT_NAME} ${PROJECT_ARGS})
# check_minimal_cxx_standard(11 ENFORCE)

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
  src/control_m.cpp
)

#3.
target_include_directories(${PROJECT_NAME} PRIVATE include)

#add_project_dependency(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.0.5")

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
pluginlib_export_plugin_description_file(${PROJECT_NAME} control_m_plugins.xml)

#6. install directives for targets and include directory.
install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(FILES
  control_m_plugins.xml DESTINATION ${PROJECT_NAME}/
)

#9. ament export libraries
ament_export_libraries(
  control_m
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  control_m
  controller_interface
  hardware_interface
  ros2_hardware_interface_bolt
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

ament_package()