In the Start we will see 3 things : 
                            -   How to have ros2 and ros2 control, 
                            -   How to create a good workspace and bolt with all there dependencies,
                            -   And how to do your first colcon build.


    1 - First step, ROS 2 :

The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what we need for the Bolt project. And it’s all open source.

ROS 2 it's the big brother of ROS 1, he adapt the changes of robotics community since the start of ROS 1 and leveraging what is great about ROS 1 and improving what isn’t 


    Download ROS2 :

        - You need a local setup with Ubuntu 20.04 
        - If you have one go to this link for ROS 2 Installation 
            [ROS 2 Foxy Installation] (https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)
        
    Download ROS2 CONTROL :

        - ROS2 CONTROL is a framework for (real-time) control of robots. This goal is to simplify integrating new hardware and overcome some drawbacks.
        
        - Do :
            -`sudo apt update`
            -`sudo apt install ros-foxy-ros2-control`
            -`sudo apt install ros-foxy-ros2-controllers`


    2 - Second step, Workspace :

You need to have a clear workspace for doing some tests or finding your files easier. Lot of different workspace exist but are more simple for the rest of the tutoriel you used the same workspace of me. 

        - Have a local workspace is better :
            `cd /users/local/<User_Name>`

        - If the path doesn't exist create them :
            `cd /`
            `mkdir users/local/<User_Name>`
            `cd users/local/<User_Name>`
            
        - Create your Workspace :

            `mkdir Bolt_ws/src`
            `cd Bolt_ws/src`

        - Add the bolt project (in the src file) :
        You need to have git tool install 

            `git clone https://github.com/stack-of-tasks/ros2_control_bolt.git`
        
        - Add some dependencies of Bolt (in the src file) :

            `git clone https://github.com/open-dynamic-robot-initiative/master-board.git`
            `git clone https://github.com/open-dynamic-robot-initiative/odri_control_interface.git`


    3 - Third step, Colcon Build  

Every time you change something in your code you need to upload with a colcon build otherwise your update won't be transmitted.

        - Go to the Bolt_ws file :

            `cd ..`

        - Do your first Colcon Build :

            `colcon build`

        If all installed properly you must have this end when you do your first colcon build :

            [IMAGE COLCON BUILD]
        
        - For update just a part of your code you changed you can do that :


            `colcon build --packages-select ros2_hardware_interface_bolt`

        This example is if you just wanna update the package ros2_hardware_interface_bolt




That's all for the first Tutorial on Bolt, I hope it could help you and go to the next Tutorial for the next part. 
