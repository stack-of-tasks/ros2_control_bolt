# ROS2_CONTROL_BOLT

## Introduction

`ros2_control_bolt` is a repository that enables controlling and simulating the Bolt robot through the [ros2_control](https://control.ros.org) infrastructure.
It allows to have bolt displayed through rviz, provides the access to the [ros2_controllers](https://github.com/ros-controls/ros2_controllers).
The most notorious is `joint_state_broadcaster` which provides the topic `/joint_states` for free.
It is then possible the node `robot_state_publisher` to have the TF-2 tree of the Bolt robot and to display it on rviz.

We provide also a gazebo plugin in order to simulate the robot in the famous Gazebo simulator.

## Install

The install procedure is described in details [here](doc/Start.md).

## Repository Organization

[doc](doc) - a set of tutorials to start, calibrate and launch Bolt.

[gazebo_ros2_control_bolt](gazebo_ros2_control_bolt) - a ROS2 package that allows to simulate Bolt robot on Gazebo and apply position, velocity, effort and gains commands to it.

[position_velocity_effort_gain_controller](position_velocity_effort_gain_controller) - a ROS2 package in which is implemented a simple controller that enables to send position, velocity, effort and gain commands to the robot, with the specific hardware interface.

[ros2_control_bolt_bringup](ros2_control_bolt_bringup) - contains files that enable to launch Bolt within its GUIs, and with it hardware interfaces and controllers.

[ros2_description_bolt](ros2_description_bolt) - contains files necessary to describe, create and visualize Bolt with Rviz and Gazebo. It stores URDF-description files, rviz configurations and meshes for the demo robots.

This repository relies on the [ros2_hardware_interface_odri](https://github.com/stack-of-tasks/ros2_hardware_interface_odri) which is a ros2_control driver for the ODRI board.

## Credits

 * Maxime-Ulrich Fansi (04/2022-09/2022) - First working version of gazebo_bolt_ros2_control
 * Benjamin Amsellem (10/2021-02/2022) - First working version of ros2_hardware_bolt
 * Paul Rouanet - (03/2021-09/2021) - Building the LAAS Bolt, starting this repo
 * Olivier Stasse (03/2021 - Supervision)
