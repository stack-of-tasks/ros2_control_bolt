# ros2_control_bolt
----------------------

Author : Paul Rouanet

### What it is

A description of physical properties of Bolt collected from ODRI (robot_properties).

The meshes of each actuators and of the body are in "meshes".

### In ros2_control :

system_bolt.ros2_control.xacro :

Definition of all data useful for system_bolt.cpp (ros2_hardware_interface_bolt) :

   - Ethernet connection name
   - Each joint with :
      - Name
      - Min and max values about position, velcity, effort, gain_kp and gain_kd
      - State values for the same data
      - Properties of each joint : offset, number, polarity ...
   - The IMU with :
      - x, y, z values of gyroscope, accelerometer, linear_acceleration, attitude_euler
      - x, y, z, and w values of quaternion

### In urdf

crane.urdf.xacro : all URDF data about the crane

leg.xacro : all URDF data about legs

system_bolt.urdf.xacro : global doc that calls leg.xacro

system_bolt_description.urdf.xacro : this doc calls ALL the others
