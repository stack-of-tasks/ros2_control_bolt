# bolt/demos

## What it is

How to use Bolt at LAAS-CNRS to do demonstrations (calibration, sensor reading and command writing), with ONTAKE computer

## Authors

- Paul Rouanet

## Goal of each code

- demo_bolt_calibration.cpp : Demo to test the calibration on real robot.

- demo_bolt_sensor_reading.cpp : Reading and printing of sensors data, without control

- demo_bolt_write_commands.cpp : Demo of a sinusoidal control on each joint : swinging effect of Bolt's legs


## How to compile at LAAS ? (on ONTAKE computer)

Put the workspace at the root, in usr. Ex : `/usr/bolt_ws`. Then, put all source codes needed in a directory named `/src`.
```
mkdir /usr/bolt_ws/src
cp <@SourceCodes> /usr/bolt_ws/src
```

Before any compilation, go in bolt_ws :
```
cd /usr/bolt_ws
```

When it is done, you are ready to compile. Always in `/usr/bolt_ws` :
```
colcon build --packages-up-to ros2_control_bolt
```

After that, put the robot in position with legs stretched (more or less in initial position), and switch on the robot : turn on the alimentation (25V, ~1A), and the emergency stop button.

Then, you can send the executable to the robot. We will take the example of demo_bolt_sensor_reading.cpp to illustrate the command to send in the shell.

```
sudo -E "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" /usr/bolt_ws/build/ros2_control_bolt/ros2_control_bolt_demo_bolt_sensor_reading enp3s0
```

Here you are, the robot will realize the action of the code sent !
