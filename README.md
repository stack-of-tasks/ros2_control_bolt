# ROS2_CONTROL_BOLT

## Introduction

`ros2_control_bolt` is a repository that enables controlling and simulating the Bolt robot.


## Repository Organization

`doc` - a set of tutorials to start, calibrate and launch Bolt.

`gazebo_ros2_control_bolt` - a ROS2 package that allows to simulate Bolt robot on Gazebo and apply position, velocity, effort and gains commands to it.

`position_velocity_effort_gain_controller` - a ROS2 package in which is implemented a simple controller that enables to send position, velocity, effort and gain commands to the robot, with the specific hardware interface.

`ros2_control_bolt_bringup` - contains files that enable to launch Bolt within its GUIs, and with it hardware interfaces and controllers.


`ros2_description_bolt` - contains files necessary to describe, create and visualize Bolt with Rviz and Gazebo. It stores URDF-description files, rviz configurations and meshes for the demo robots.

`ros2_hardware_interface_bolt` - a ROS2 package in which a hardware interface for Bolt is implemented.


## Quick Hints

These are some quick hints, especially for those coming from a ROS1 control background:

* There are now three categories of hardware components: *Sensor*, *Actuator*, and *System*.
  *Sensor* is for individual sensors; *Actuator* is for individual actuators; *System* is for any combination of multiple sensors/actuators.
  You could think of a Sensor as read-only.
  All components are used as plugins and therefore exported using `PLUGINLIB_EXPORT_CLASS` macro.
* *ros(1)_control* only allowed three hardware interface types: position, velocity, and effort.
  *ros2_control* allows you to create any interface type by defining a custom string. For example, you might define a `position_in_degrees` or a `temperature` interface.
  The most common (position, velocity, acceleration, effort) are already defined as constants in hardware_interface/types/hardware_interface_type_values.hpp.
* Joint names in <ros2_control> tags in the URDF must be compatible with the controller's configuration.
* In ros2_control, all parameters for the driver are specified in the URDF.































