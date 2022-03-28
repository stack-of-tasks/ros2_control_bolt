# FORWARD POSITION

After you've followed all previous tutorials, you have the required knowledge to moove Bolt in position. For that, you have to follow 2 steps :

- Set the positions 
- Launch the file 


## Set the positions

You can use a controller for moove bolt where ever you want, for that you just need some points of each joints. For find points you wanna send to bolt you can run the file [demo_bolt_sensor_reading.cpp](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_hardware_interface_bolt/test/demo_bolt_sensor_reading.cpp):

- Open a Terminal and source Bolt :

        source install/setup.bash
        
- Run the demo sensor reading file :

        ros2 run --prefix="sudo -E env PATH=${PATH} LD_LIBRARY_PATH=${LD_LIBRARY_PATH} PYTHONPATH=${PYTHONPATH}" ros2_hardware_interface_bolt demo_bolt_sensor_reading
      
- Moove the robot where you wanna he go and do a `Ctrl-C` for stop the file `demo_bolt_sensor_reading` running.

- Copy the last value in your Terminal and paste it in the file [bolt_forward_position_publisher.yaml](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_bringup/config/bolt_forward_position_publisher.yaml), at `line 8` `pos1`

- Repeat that how many times you want, just past the next position in the file yaml at the pos2 ect...

- If you need more than 4 position, you can add some but you need to add it in a `goal_names[]` `line 7` and add a line for the position, for exemple pos5.

- You can also play with the value `wait_sec_between_publish` `line 5`, for Bolt be faster reduce it and for Bolt be slower raise it.

- Now you have create your own cycle, you need to launch the file for see Bolt mooves.  


## Launch file test_forward_position_controller

When you have set up all the positions, you need to run the new controller :

- Open a new terminal, source ros and do a colcon build if you have change something

- Run in this terminal the `bolt_system_position_only` file (if you don't know how, follow the previous tutorial) :

      ros2 launch ros2_control_bolt_bringup bolt_system_position_only.launch.py
      
- When it running you have 2 way to run the `bolt_forward_position_publisher` **controller** :

    - First, look in the file `bolt_system_position_only` at `DeclareLaunchArgument` line 54 and see the default_value. Normally, if haven't change anything you will see `forward_position_controller`, and it's the controller we need to launch, it is already launch in the file we just need to launch the file :
        
        - Open a new Terminal, source ros

        - You can see the controller is already started :

                      ros2 control list_controllers

                      [PHOTOS LIST_CONTROLLERS]

        - You just need to launch the file :

                      ros2 launch ros2_control_bolt_bringup test_forward_position_controller.launch.py

          **Now you can see bolt go to each points you have give to it.**
            
   
    - Second, if you haven't the `forward_position_controller` to `default_value` for the `DeclareLaunchArgument` and you don't want to put them for some reason. You can start the controller in an other method, **manually** :
    
      - Open a new Terminal, source ros and do :
        
            ros2 control load_controller forward_position_controller
        
      - Check if the controller is loaded properly:

            ros2 control list_controllers
            
      - Then configure it:

            ros2 control set_controller_state forward_position_controller configure

      -  Check if the controller is loaded properly:

              ros2 control list_controllers

          You should get the response:

              forward_position_controller[forward_command_controller/ForwardCommandController] inactive
                
      - Now start the controller:

              ros2 control switch_controllers --start forward_position_controller

      - Check if controllers are activated:

              ros2 control list_controllers
                
         You should get active in the response:

              joint_state_controller[joint_state_controller/JointStateController] active
              forward_position_controller[forward_command_controller/ForwardCommandController] active
                
      - Now the controller is active, you can launch it :

              ros2 launch ros2_control_bolt_bringup test_forward_position_controller.launch.py
              
        Now you can see bolt go to each points you have give to it.
        
    
**Becarefull when you start the controller you always need to have the emergency stop button nearly**

### Thank you for having followed all  the tutorial and good luck.
