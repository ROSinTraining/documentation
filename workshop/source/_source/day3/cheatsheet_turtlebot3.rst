Cheatsheet: Turtlebot3
======================

General Tips
---------------------
Please always check if someone is already using a TurtleBot by e.g.:

.. code-block:: bash 

    $ rosnode list

Export EnvironmentVariables
---------------------------
This has to be done in **every** terminal!

1. Youneed to set the TurtleBot model before using any *.launch* files:

    .. code-block:: bash 

        $ export TURTLEBOT3_MODEL=waffle# for waffle
        $ export TURTLEBOT3_MODEL=burger# or for burger

2. Setup the ROS network variable(if you interact with the real robot):


    .. code-block:: bash

        $ export ROS_MASTER_URI=http://<TB-IP>:11311
        $ export ROS_IP=<MY-IP>

    For example:

    .. code-block:: bash

        $ export ROS_MASTER_URI=http://192.168.1.199:11311
        $ export ROS_IP=192.168.1.10

    To find the IP address of a computer use the command:

    .. code-block:: bash

        $ ifconfig

Start the Turtlebot3
--------------------------

1. Using the simulation:

    .. code-block:: bash

        ## Setthe TURTLEBOT3 MODEL. See “2.”
        $ roslaunch turtlebot3_gazebo turtlebot3_world.launch

2. Real robot(after exporting the environment variables):
   Check if the TurtleBot is already started. See “1. General TIPS”.
   
   *ON THE TURTLEBOT COMPUTER*:
    * Remotely connect to the TurtleBot
       
       .. code-block:: bash
           
           ## Setthe TURTLEBOT3 MODEL. See “2.”
           $ roslaunch turtlebot3_gazebo turtlebot3_world.launch

    * Bring upthe TurtleBotdrivers
       
       .. code-block:: bash
           
           $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
    
    *ON YOUR REMOTE COMPUTER*:

    * Load the robot description into the ROS system
       
       .. code-block:: bash
           
           ## Setthe TURTLEBOT3 MODEL. See “2.”
           $roslaunch turtlebot3_bringup turtlebot3_remote.launch
        
    All commands of this page are supposed to be started on yourremotecomputer.

Control the robot
--------------------------

* Via Keyboard

    .. code-block:: bash
    
        $roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
* With the RQT plugin
  
    .. code-block:: bash
        
        $ ros run rqt_robot_steering rqt_robot_steering

* Using a PS3 Joystick

    .. code-block:: bash
    
        $ roslaunch teleop_twist_joy teleop.launch

* Visualization using RVIZ (in a new terminal):

    .. code-block:: bash
        
        $ rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz
        # Watch out to use backticks ` and not quotation sign ‘.

SLAM
----------------------

* Make sure that either the simulation or the real robot is started! See “3.”
* Start SLAM (using gmapping):

    .. code-block:: bash
        
        ## Set the TURTLEBOT3 MODEL. See “2.”
        $ roslaunch turtlebot3_slam turtlebot3_slam.launch

* Save the map:

    .. code-block:: bash
        
        $ rosrun map_server map_saver -f ~/map

Navigation
-----------------------

* Make sure that either the simulation or the real robot is started! See “3.”
* Using the navigation:

    .. code-block:: bash
        
        ## Set the TURTLEBOT3 MODEL. See “2.”
        $roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
