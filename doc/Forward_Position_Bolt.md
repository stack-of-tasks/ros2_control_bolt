# FORWARD POSITION

After you've followed all previous tutorials, you have all the knowledge required to change the position of BOLT. To do this, you must follow 2 steps:
- Set the positions
- Launch the file


## Set the positions


To move Bolt as you wish, you can use a `controller`. For this you will just need a few points from each joint. To find the points you want to send to Bolt, you can use the file [demo_bolt_sensor_reading.cpp](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_hardware_interface_bolt/test/demo_bolt_sensor_reading.cpp):

1) Open a Terminal, go to your `Bolt_ws/` workspace and source ROS :

        source /opt/ros/foxy/setup.bash

        source install/setup.bash

2) Run the demo sensor reading file :

        bolt_config_path=src/ros2_control_bolt/ros2_description_bolt/config/bolt_config.yaml

        ros2 run --prefix="sudo -E env PATH=${PATH} LD_LIBRARY_PATH=${LD_LIBRARY_PATH} PYTHONPATH=${PYTHONPATH}" ros2_hardware_interface_odri demo_bolt_sensor_reading $bolt_config_path

3) Move the robot to where you want it to go and press `Ctrl-C` to stop the file `demo_bolt_sensor_reading` running.

4) Copy the last value in your Terminal and paste it in the file [bolt_forward_position_publisher.yaml](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_bringup/config/bolt_forward_position_publisher.yaml), at `line 8`
`pos1`

5) Repeat this as many times as you want , just pass the next position in the yaml file to pos2 etc...

- If you need more than 4 positions, you can add some extra ones, but you must add them in the following order a `goal_names[]`
`line 7` and `add a line for the position`, for example pos5.

- You can also play with the value `wait_sec_between_publish` `line 5`. For Bolt to be faster reduce it and for Bolt be slower raise it.

- Now that you have created your `own cycle`, you have  to `launch` the file to see Bolt moving.


## Launch file test_forward_position_controller
When you have configured all the positions, you should run the new controller. :

1) Open a new terminal, go to your `Bolt_ws/`, source ROS and do a colcon build if you have changed anything.


2) Run in this terminal the `bolt_system_position_only` file (if you don't know how, follow the previous tutorial) :

        ros2 launch ros2_control_bolt_bringup bolt_system_position_only.launch.py

3) When it is running you have 2 ways to run the `bolt_forward_position_publisher` **controller** :

   3.1) First, look in the file `bolt_system_position_only` at `DeclareLaunchArgument` line 54 and see the `default_value`. Normally, if you haven't changed anything, you'll see `forward_position_controller`, and that's the controller you need to run, it's already running in the file, you just need to `run the file` :

   - Open a new Terminal, go to your `Bolt_ws/` workspace and source ROS

   - You can see that the controller is already running:

                      ros2 control list_controllers

                      [PHOTOS LIST_CONTROLLERS]

   - You just have to launch the file :

        ros2 launch ros2_control_bolt_bringup test_forward_position_controller.launch.py

          **Now you can see bolt going to every point you gave it.**


   3.2) Secondly, if you haven't the `forward_position_controller` to `default_value` for the `DeclareLaunchArgument` and you don't want to put it on for whatever reason . You can start the controller by another method, **manually**:

   - Open a new Terminal, source ros and do :

              ros2 control load_controller forward_position_controller

   - Check if the controller is loaded properly:

              ros2 control list_controllers

   - Then configure it:

              ros2 control set_controller_state forward_position_controller configure

   - Check if the controller is loaded properly:

              ros2 control list_controllers

       You should get the response:

              forward_position_controller[forward_command_controller/ForwardCommandController] inactive

   - Now start the controller:

              ros2 control switch_controllers --start forward_position_controller

   - Check if the controller is activated:

              ros2 control list_controllers

       You should get active in the response:

              joint_state_controller[joint_state_controller/JointStateController] active
              forward_position_controller[forward_command_controller/ForwardCommandController] active

   - Now that the controller is active, you can launch it :

              ros2 launch ros2_control_bolt_bringup test_forward_position_controller.launch.py

       Now you can see the bolt go to each point you gave it.


**Be careful, when you start the controller, you must always have the emergency stop button nearby.**

If you are tired of sourcing ros2 every time you open a new terminal, you can add the following commands in your `/home/<User_name>/.bashrc` file :

        cd /users/local/<User_Name>/Bolt_ws/
        source /opt/ros/foxy/setup.bash
        source install/setup.bash

Those three commands will be return every time you open a new terminal.

### Thank you for following the entire tutorial and good luck.
