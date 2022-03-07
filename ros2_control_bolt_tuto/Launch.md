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

1)  Problem with a phantom node :

    - If you do this command when nothing is run 
