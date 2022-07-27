# ros2_hardware_interface_odri
----------------------

## Introduction

This project provides a ros2_control SystemInterface to the master board ODRI.
The particularity of this board is to provide a command interface in position, velocity, torque and gains (Kp, Kd).

This package can be used together with a description package in order to provide the mapping between the robot models
and the actuator. A robot_config.yaml file is needed.

## Installing from source:

```
colcon build --packages-select ros2_hardware_interface_odri
source ./install/setup.bash
ros2 launch ros2_control_odri_bringup system_odri.launch.py
```

## How to use it ?

### ros2_control

In your xacro file (used to generate the urdf file) you can add the following snippet:
```
 <plugin>ros2_control_bolt/SystemBoltHardware</plugin>
            <param name="bolt_yaml">2.0</param>
            <xacro:property name="prop_bolt_config_yaml" value="$(find ros2_description_bolt)/config/bolt_config.yaml" />
            <param name="bolt_config_yaml">${prop_bolt_config_yaml}</param>
	    <param name="desired_starting_position">0.0 0.0 0.0 0.0 0.0 0.0</param>
```

The field ```desired_starting_position``` should have the same size than the number of joints.

### In system_odri :

Inclusion of all odri_control_interface header needed to use Odri.

Use of the namespace ros2_control_odri

Define of a structure used for IMU : GyroAccLineEulerQuater (must be optimized (x,y,z))


Functions :
   - Definition of init_robot: This function use ODRI methods to initialize the robot :
      - Call Ethernet output
      - Define main_board_ptr_ as theMasterboard with the Ethernet name
      - Define all the joints_ with properties
      - Define the IMU
      - Finally define the robot_ with those 3 elements

   - calibration() : use to calibrate the robot. Currently called in start function. **Maybe a bug to fix line 532 :  robot_->RunCalibration(calib_ctrl); {namespace ?}

   - start() : Start the robot, set some default values to 0, do the calibration, read sensors data

   - stop() : stop the Masterboard, so stop the robot

   - read() : sed all the sensors data to the computer

   - write() : allow the user to send commands to all actuators


----------------------

### Tests :

The compilation with ```colcon build --packages-select ros2_hardware_interface_odri``` is ok



Tested on Bolt.

# Credits :

 *   Maxime-Ulrich Fansi (04/2022-09/2022) - First working version of gazebo_bolt_ros2_control
 *   Benjamin Amsellem (10/2021-02/2022) - First working version of ros2_hardware_bolt
 *   Paul Rouanet - (03/2021-09/2021) - Building the LAAS Bolt, starting this repo
 *   Olivier Stasse (03/2021 - Supervision)
