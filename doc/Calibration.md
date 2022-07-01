# CALIBRATION

After you've followed how to Start and Setup Bolt, you need to know how to Calibrate Bolt, that will take you 3 steps :

- What is Calibration and how it works ?
- How to find the Offset and set them
- How to start some test with Bolt


## 1- What is Calibration and how it works

Calibration is the step where the robot finds a point that we have predetermined. Every time we turn Bolt off, he loses track of his position, because he doesn't know if these motors should move or not. To have an initial position that we predefine, each motor had to hit an index that it knew this position. The Calibration is the part where the robot hits an index for each motor and says, it's my 0,0 after he has taken the offset we have configured and go back to his position and says now it's my real 0,0. He does that every time we turn it off.

For each motor, we define a way for it to find  its Index :

  -  `POS` : the motor search the first index in his **positive** axe
  -  `NEG` : the motor search the first index in his **negative** axe
  -  `ALT` : the search is in **positive and negative** axe
  -  `AUTO` : find **automatically** the way (positive or negative) some bugs may occur

Calibration is called when you launch the CPP file and when you call the function initialize.

## 2- Find the Offset and set them

For this step, you must have configured all the parameters mentioned in the previous tutorials.

1) Stop Bolt, put the robot's legs as straight as possible and turn Bolt on.

2)  Open a new Terminal and source ROS

3)  Run in your Terminal :

        ros2 run --prefix="sudo -E env PATH=${PATH} LD_LIBRARY_PATH=${LD_LIBRARY_PATH} PYTHONPATH=${PYTHONPATH}" ros2_hardware_interface_bolt bolt_hardware_calibration

4)  Place the calibration object on Bolt while the simulation is running.

    ![Bolt With Object](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Calibration_Bolt_1-R.jpeg?raw=true "Bolt with object")
    ![Bolt With Object](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Calibration_Bolt_2-R.jpeg?raw=true "Bolt with object")

5) When the object is placed, press `Ctrl + C` to stop the simulation and take one of the latest return values.

    ![Bolt Calibration Values](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Calibration_Bolt_3-R.jpeg.png?raw=true "Bolt Calibration Values")

6)  Open file `ros2_control_bolt/ros2_description_bolt/src/bolt_config.yaml` and paste the values in the variable **position_offsets (line 30)**, save the file and do a colcon build at `Bolt_ws/`.

    ![Offset Change](https://github.com/Benjamin-Amsellem/ros2_control_bolt/blob/master/ros2_control_bolt_tuto/pictures/Calibration_Bolt_4-R.png?raw=true "Offset Change")


7)  If you want to change the way each motor finds its Index, go to line 26 at Search_methods  and change it as you wish (by default, everything is set to POS):

      - `POS`;
      - `NEG`;
      - `ALT` or
      - `AUTO`.

Now that everything is ready for the calibration, follow the next step to try some tests.

## 3- Some Tests

You know a lot of things about Bolt, but now let see the change you have operated.

1) Open a Terminal, source your ROS

3) If you have done some modification, run a Colcon Build always in `Bolt_ws/`, never in the `Bolt_ws/src/`

8)  For example, run file `demo_bolt_from_yaml.cpp` and observe Bolt be positioned at the position calibrated :

   		   ros2 run --prefix="sudo -E env PATH=${PATH} LD_LIBRARY_PATH=${LD_LIBRARY_PATH} PYTHONPATH=${PYTHONPATH}" ros2_hardware_interface_bolt demo_bolt_from_yaml

    **Now you know how to Start Bolt and have a good Calibration.**

### Next step is to see how to use ros2 launch and have a modeling of Bolt in Rviz.
