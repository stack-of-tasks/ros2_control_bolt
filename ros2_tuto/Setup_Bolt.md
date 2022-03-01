# BOLT TUTORIAL :
## SETUP

After you've follow the the Start tutorial you need to know how to setup Bolt, that will take you 3 steps :

* Bolt skeleton
* How to plug correctly Bolt
* How to setup somes informations for bolt 



##  1)  Bolt Skeleton :

Bolt is a tiny robot completely built in our labs. We have assemblate their components, modeled their bones with a 3D printer and made it look like real robots. 

  - If one day, one of their bones broke you will need to go to this link and follow the tutorial. You will have to print a new bone for Bolt with a 3D Printer :
      - ' TUTORIAL FOR BOLT 3D PRINTING '


##  2)  Plug Bolt properly :

1) You need before any commands who need Bolt have it properly connected. Is really simple that what you need :

    - An alimentation,
    - A pc with ubuntu 20.04 and 2 or more ethernet port,
    - An emergency stop button,
    - A power cable. 

2) Take your alimentation, put them on and adjust behind alimentation the port (P1/P2/P3) and put the mode to SET then set the Voltage to 20V and amperage 5A. Finally change the mode to NORMAL.

        [PHOTO DERRIERE L'ALIMENTATION]

3) Connect the emergency stop button to the alimentation (by behind not front).

        [PHOTO DU BRANCHEMENT DEVANT BARRE] [PHOTO DU BRANCHEMENT DERRIERE]

4) Connect the power cable to Bolt, one end to the ethernet port of the Master-Board and the power cable inside Bolt.

        [PHOTO DE LA CONNECTION A BOLT]

5) See if any cable is plugged out of the Master-Board (below Bolt).

        [PHOTO DES BONS BRANCHEMENTS]

6) Connect the other end to the emergency button and the ethernet cable to your pc. 

        [PHOTO DE LA CONNECTION AU BOUTON]


7) Finally unpress the emergency stop button and power on the alimentation.

   - If you see a Red-Light flash below Bolt, it means Bolt is ON


**Now Bolt is connected properly to your computer.**


## 3) Set some important information on Bolt :

1) You need to set the internet port of Bolt in the code. 

   - Open a Terminal and run :

          ifconfig

2) All the name ports start with "en" you can see them on the left of your terminal.

    - Power ON Bolt.

    - Open a new Terminal go in your Bolt_ws file and source your ROS 2 :

          source install/setup.bash'

     - Try one by one all the port with this command,  <PORT> is a value where you change the port name :

            ros2 run --prefix="sudo -E env PATH=${PATH} LD_LIBRARY_PATH=${LD_LIBRARY_PATH} PYTHONPATH=${PYTHONPATH}" ros2_hardware_interface_bolt demo_bolt_sensor_reading <PORT>
3) When you haven't an error and you have some values returned you have found the correct port.    

      - Copy paste the right port in the file :
  
             Bolt_ws/src/ros2_control_bolt/ros2_hardware_interface_bolt/test/config_bolt.yaml 
        at :
  
              interface : <PORT>     (line 4) 

4) Save your file and now you don’t need to put the port name in every command you send to Bolt. 

    **Now you have seen how you put the correct ethernet Port in the code.**
  
### Your next step is to run some tests to see how the Bolt works.
