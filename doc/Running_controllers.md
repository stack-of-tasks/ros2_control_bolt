# Controlling Bolt

After you've followed all previous tutorials, you have all the knowledges required to change the position of BOLT.
In this tutorial, we will show you how to send commands to Bolt.

## Sending position commands to Bolt
When you have configured all the positions, you should run the new controller. :

1) Open a new terminal, go to your `Bolt_ws/`, source ROS and do a colcon build if you have changed anything.


2) Run in this terminal the `bolt_system_position_only` file (if you don't know how, follow the previous tutorial) :

       ros2 launch ros2_control_bolt_bringup bolt_system_position_only.launch.py

3) For the moment, when it is running you have 2 ways to send commands by publishing in a topic :

   3.1) First, look in the file `bolt_system_position_only` at `DeclareLaunchArgument` line 54 and see the `default_value`. Normally, if you haven't changed anything, you'll see `forward_position_controller`, and that's the controller you need to run, it's already running in the file, you just need to `run the file` :

   - Open a new Terminal, go to your `Bolt_ws/` workspace and source ROS

   - You can see that the controller is already running:

              ros2 control list_controllers

              [PHOTOS LIST_CONTROLLERS]

   - Then you just need to run this command in the terminal :

```
ros2 topic pub /forward_command_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- -0.5
- 0.0
- 0.0
- 0.0
- 0.0"
```

   **Now you can see bolt moving accordingly.**

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

   - Now that the controller is active, you can run this command :

```
ros2 topic pub /forward_command_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- -0.5
- 0.0
- 0.0
- 0.0
- 0.0"
```

   **Now you can see bolt moving accordingly.**
          
## Controlling Bolt position, velocity, effort and gains at the same time

Doing that is almost similar to the previous part. The differences are :
   - You need a specific controller [position_velocity_effort_gain_controller](https://github.com/stack-of-tasks/ros2_control_bolt/tree/master/position_velocity_effort_gain_controller)
   - You need to send 30 values instead of just 6 for position control.

`position_velocity_effort_gain_controller` includes :

   - a `config` folder which is made of [bolt_pveg_controller.yaml](https://github.com/stack-of-tasks/ros2_control_bolt/blob/master/position_velocity_effort_gain_controller/config/bolt_pveg_controller.yaml) in which you specify 
     your pveg_controller, all the joints you want to control (6 in this case), their order, and the 5 interfaces you want to use : position, velocity, effort, gain_kp and gain_kd
   
   - a `launch` folder which is made of launch files that allows you to launch Bolt with Rviz and test it in real-time.
   
As previously, you have two ways to use `position_velocity_effort_gain_controller` :

1) If `pveg_controller` is specified in the file `bolt_system_pveg.launch.py` at `DeclareLaunchArgument` line 54 in the `default_value`, run this command :

       ros2 launch gazebo_ros2_control_bolt bolt_system_position_only_gazebo.launch.py 
        
2) If not :

   - Open a new Terminal, go to your `Bolt_ws/` workspace, source ROS and load the controller :

         ros2 control load_controller pveg_controller

   - Check if the controller is loaded properly:

         ros2 control list_controllers

       You should get the response:
       
         joint_state_controller[joint_state_controller/JointStateController] active
         pveg_controller[position_velocity_effort_gain_controller/PosVelTorGainsController]inactive        

   - Then configure it:

         ros2 control set_controller_state pveg_controller configure

   - Check if the controller is loaded properly:

         ros2 control list_controllers

       You should get the response:

         joint_state_controller[joint_state_controller/JointStateController] active
         pveg_controller[position_velocity_effort_gain_controller/PosVelTorGainsController]inactive

   - Now start the controller:

         ros2 control switch_controllers --start pveg_controller

   - Check if the controller is activated:

         ros2 control list_controllers

       You should get active in the response:

         joint_state_controller[joint_state_controller/JointStateController] active
         pveg_controller[position_velocity_effort_gain_controller/PosVelTorGainsController] active
         
 
If another controller, [name]_controller, is specified in the in the `default_value`, and if this controller claims the same interfaces as yours,
there will be conflicts. To solve them, you have to :
    
   - Stop [name]_controller :
   
         ros2 control switch_controllers --stop [name]_controller
    
   - Then load pveg_controller :
   
         ros2 control load_controller pveg_controller
        
   - Configure it :
   
         ros2 control set_controller_state pveg_controller
        
   - And then start it :
   
         ros2 control switch_controllers --start pveg_controller
              
3)At this point, everyting should work correctly. You can now publish some data by running this [script](https://github.com/Maxime-Fansi-laas/ros2_control_bolt/blob/master/ros_command_interface_script.sh) (make sure to be in your workspace `Bolt_ws/`):

    source src/ros2_control_bolt/ros_command_interface_script.sh
              
If you look closely at it, you'll see that there are 30 values divided in 5 blocks (interfaces) of 6 values (joints). The first block is for positions, the second for velocities,
the third for torques, the fourth for kp gains and the fifth for kd gains. You'll also see that everything is set to 0. 

This is because we don't know yet what values to send to the robot without crashing it. But it is possible to do some tests with Gazebo. If you have understood this tutorial, 
try to run pveg_controller on Gazebo.

If you want the robot to move (on Gazebo or in real-time), you just need to change those values. However, tests them on Gazebo before applying them on the real robot.


   **Now you can see bolt moving accordingly.**
   

**Be careful, when you start the controller, you must always have the emergency stop button nearby.**

If you are tired of sourcing ros2 everytime you open a new terminal, you can add the following commands in your `/home/<User_name>/.bashrc` file :

    cd /users/local/<User_Name>/Bolt_ws/
    source /opt/ros/foxy/setup.bash
    source install/setup.bash
        
Those three commands will be run everytime you open a new terminal.

### Thank you for following the entire tutorial and good luck.
