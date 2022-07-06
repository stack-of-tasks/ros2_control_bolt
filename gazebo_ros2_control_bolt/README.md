# gazebo_ros2_control_bolt

gazebo_ros2_control_bolt is a ROS2 package that allows to simulate Bolt robot on Gazebo and apply position,
velocity and force commands to it.

## Quick start

To start Gazerbo with bolt you can start with:
`ros2 launch gazebo_ros2_control_bolt bolt_system_position_only_gazebo.launch.py`

A Gazebo client window should open.

Then to control the robot towards a specific configuration:
```
ros2 topic pub /forward_command_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- -0.5
- 0.0
- 0.0
- 0.0
- 0.0"
```

This should give the following output:
<img src="./doc/pictures/gazebo_bolt_position_controlled.jpg" width="800" alt="Display " align="center"/>
