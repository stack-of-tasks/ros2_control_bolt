#!/bin/bash

# This script aims at sending commands to Bolt by publishing data on 
# /pveg_controller/commands topic.

reset

#pveg command
ros2 topic pub /pveg_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0

- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0

- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0

- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0

- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0

"

