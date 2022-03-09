# CALIBRATION

After you've followed how to Start and Setup Bolt, you need to know how to Calibrate Bolt, that will take you 3 steps :

- What is Calibration and how it works ?
- How to find the Offset and set them
- How to start some test with Bolt 


## 1- What is Calibration and how it works 

The Calibration is the step when the robot finds a point that we have predetermine. Each time we turn off Bolt, he lost their notion of position, because he doesn't know if these motors have to move or not. For having an initial position we predefine, each motor need to hit an index who he knew this position. The Calibration is the part where the robot hit an index for each motor and say to him, it's my 0,0 after he takes the offset we have configured and go to their position and say now it's my real 0,0. He does that every time we put it off. 

For each motor, we define a way for how they find their Index :

  -  `POS` : the motor search the first index in their **positive** axe 
  -  `NEG` : the motor search the first index in their **negative** axe
  -  `ALT` : the search is in **positive and negative** axe
  -  `AUTO` : find **automatically** the way (positive or negative) can occur some bugs
  
Calibration is call when you launch the CPP file and when you call the function initialize. 


## 2- Find the Offset and set them 

For this step, you need to have configured all the parameters you need mentioned in the previous tutorials. 

1)  Shut down Bolt, put the robot legs as straight as possible and turn on Bolt

2)  Open a new Terminal and source ROS   

3)  Run in your Terminal :

        ros2 run --prefix="sudo -E env PATH=${PATH} LD_LIBRARY_PATH=${LD_LIBRARY_PATH} PYTHONPATH=${PYTHONPATH}" ros2_hardware_interface_bolt bolt_hardware_calibration
       
4)  Put the calibrating object on Bolt when the simulation is on

        [PHOTO DE BOLT AVEC L'OBJET]
        
5)  When the object is put, do a Ctrl + C to stop the simulation and take one of the latest return values.

        [PHOTO DES VALEURS RETOURNE DU TERMINAL]
        
6)  Open file ros2_control_bolt/ros2_hardware_bolt/src/config_bolt.yaml and paste the values in the variable position_offsets (line 30), save the file and do a colcon build at `Bolt_ws/`.

        [PHOTO DE OU LE METTRE]
        
7)  If you want to change the way how to each motor find their Index, go to `line 26` at `Search_methods` and modify how you want (By default all is set at POS) :

      - `POS`;
      - `NEG`;
      - `ALT` or 
      - `AUTO`.
       
Now all is ready for the calibration, follow the next step to try some test.

## 3- Some Tests

You know a lot of things about Bolt, but now let see the change you have operated. 

1) Open a Terminal, source your ROS 

3) If you have done some modification, run a Colcon Build always in `Bolt_ws/`, never in the `Bolt_ws/src/` 

8)  For example, run file demo_bolt_from_yaml.cpp and observe Bolt be positioned at the position calibrated :  
   
   		   ros2 run --prefix="sudo -E env PATH=${PATH} LD_LIBRARY_PATH=${LD_LIBRARY_PATH} PYTHONPATH=${PYTHONPATH}" ros2_hardware_interface_bolt demo_bolt_from_yaml
    **Now you know how to Start Bolt and have a good Calibration.**

### Next step is to see how to use ros2 launch and have a modeling of Bolt in Rviz. 