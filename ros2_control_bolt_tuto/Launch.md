# LAUNCH

After you've follow all previous tutorials, you have the knowledge required for do your first `ros2 launch`. For that you need to follow 3 steps :

- How launch file works ?
- How to setup some information needed for the Launch
- How to rune a Launch file


## Launch file 

Launch files are code in python, we make them for create comands lines and launch some nodes. They are really usefull because when we run them we can run lot of nodes and comands lines at the same time. 

All the launch files can be find in the [ros2_control_bringup], we can run them separatly. For example we go see the file [bolt_system_position_only.launch.py]:

When you start a launch file you need to declare the description of your launch and inside we declare every argument we need. 
After you have declare every argument we initialize them with what we want to do.
At end we launch our argument, here we run a other launch file call [bolt.launch.py]. And in this launch we see the same structur and at the end we run 5 differents Nodes and run some comands. 

**If you need to launch a real time node, you need to call them in Sudo user.**

## Setup some command for Launch File

You can need to setup 2 things for run a launch file :

1.  Problem with a ghost node :

    - If you do this command when nothing is run :

		    ros2 node list 

       And you see a ghost node (if nothing is running you need to see nothing, if the command return something it's a ghost). You need to add that in your bash.rc :

		        export ROS_DOMAIN_ID=10


2.  Sudo / User talk :

    When you run a launch file with a node in sudo (real time), the sudo user and your user can't talk to each other. For resolve this problem you need to add a file inside your directory. 
    
    Don't panic the file is already added in the bolt directory [FastRTPS.xml]. But you need to call him every time you run a launch file, for that add this command in your bashrc :

		    export FASTRTPS_DEFAULT_PROFILES_FILE=<PATH_TO_THE_FILE>

    Where <PATH_TO_THE_FILE> is your path to the file FasrRTPS.xml from the root "cd /".

    After that you know every thing and you have set every thing for run your first launch file. 


## Run Launch File 

Now you can really play with Bolt, if you have follow correctly each tutorial now you can run a Launch File. 

1)  Open a new Terminal and source ros, go on the Bolt_ws directory :
	
	    source install/setup.bash

2)  Colcon build if you have do some modifications, open a new Terminal go on Bolt_ws directory and do :

	    colcon build

3)  Run your first Ros2 launch, in the Terminal where you have source Ros, do :

	    sudo ls

	    ros2 launch ros2_control_bolt_bringup bolt_system_position_only.launch.py

    You need to do a sudo ls first because you need to be already in a sudo mode. 
    
    **ALWAYS be ready to push the emergency button if the robot do a bad moove or for other emergency**

4) Normally rviz it's open, you can see the robot in real time. 

    Play with it for see all these fonctionality. 

5) You can run a topic for see all the parameters of Bolt :

    - open a new Terminal, source ros and do :

            ros2 topic list 

        you will see that list :

            [IMAGE ROS2 TOPIC LIST]

    -  run the topic /joint_states :

	        ros2 topic echo /joint_states

    **You can see all the parameters of Bolt in real time !**

    If you want you can play with all others Topics for see what they published. 

### Thank you for have follow all the tutorial and good work. 

