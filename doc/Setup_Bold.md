# SETUP


 After  having followed the Starting tutorial, you need to know how to setup Bolt, this will take you 3 steps :


* Bolt skeleton
* How to plug correctly Bolt
* How to setup some information for bolt



## 1 - Bolt Skeleton :


 Bolt is a small robot that has been totally built in our laboratories. We assembled its components, modeled its bones with a `3D printer` and gave it the appearance of real robots.


- If ever a `bone` in his body should `break`, you will have to go to this link and follow the tutorial. You will then have to `print a new bone` for Bolt with a 3D Printer :

     - ' TUTORIAL FOR BOLT 3D PRINTING '

## 2 - Plug Bolt properly :

1) For any command that needs Bolt, you must first have it properly connected. The process is simple, and for that you would need:

    - A `power supply`,
    - A pc with `Ubuntu 20.04` and `2 or more Ethernet port`,
    - An `emergency stop button`,
    - A `power cable`.

2) Take your `power supply`, turn it on and behind the power supply, adjust the port (P1/P2/P3) and set the mode to `SET` then set the voltage to `20V` and the amperage to `5A` on the front. Finally, set the mode to `NORMAL`.

![Behind alimentation](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Setup_Bolt_1-R.jpeg?raw=true "Behind alimentation")

3) Connect the `emergency stop button` to the `power supply` (from behind,and not in front) :

    ![Not Front](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Setup_Bolt_2-R.jpeg?raw=true "Not Front")
    ![Behind](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Setup_Bolt_3-R.jpeg?raw=true "Behind")

4) Connect the other end of the `emergency button` to the robot and the `Ethernet cable` to your computer.

     ![Button Connection](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Setup_Bolt_7-R.jpeg?raw=true "Buttun Connection")

5) Connect the `power cable` to `Bolt`, one end to the Ethernet port of the `Master-Board` and the `power cable inside Bolt`.

    ![Bolt Connection](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Setup_Bolt_4-R.jpeg?raw=true "Connection")

6) Check if any cable is `plugged out` of the `Master-Board` (below Bolt).

    ![Connection Master-Board](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Setup_Bolt_5-R.jpeg?raw=true "Master-Board Connection")

7) Finally, `unlock the emergency stop button` and `power on` the `supply power`.

   - If you see a `Red-Light flash` below Bolt, it means Bolt is **ON**


**Now Bolt is connected properly to your computer.**


## 3 - Set some important information on Bolt :

1) You need to set the `internet port` of Bolt in the code.

   - Open a Terminal and run :

        ifconfig

2) All the ports names start with `"en"`. You can see them on the left of your terminal. You will have to try them
in order to find the correct port. To do that :

   - Power ON Bolt.

   - Open a new Terminal, go to your `Bolt_ws` workspace and `source` your ROS 2 :
          
        source /opt/ros/foxy/setup.bash
          
        source install/setup.bash

   - Copy and past the name of a port in the file :

             Bolt_ws/src/ros2_control_bolt/ros2_description_bolt/config/bolt_config.yaml
        at :
             interface : <PORT_NAME>     (line 4)

   - Save the file and run :
   
        colcon build
             
            
3) Then run the following commands (you have to be in your `Bolt_ws/` workspace) :

        bolt_config_path=src/ros2_control_bolt/ros2_description_bolt/config/bolt_config.yaml
             
        ros2 run --prefix="sudo -E env PATH=${PATH} LD_LIBRARY_PATH=${LD_LIBRARY_PATH} PYTHONPATH=${PYTHONPATH}" ros2_hardware_interface_odri demo_bolt_sensor_reading $bolt_config_path
             
   - The correct port is found when you have some `returned values` and no errors. You should also see Bolt moving. If it doesn't work, try another port.
   
   - To stop the execution, press `Ctrl-C`.
            


4) Now you don’t need to put the port name in every command you send to Bolt.

    **Now you have seen how you put the right Ethernet Port in the code.**

### Your next step is to run some tests to see [how Bolt works](https://github.com/Maxime-Fansi-laas/ros2_control_bolt/blob/maxime_branch/doc/Calibration.md).
