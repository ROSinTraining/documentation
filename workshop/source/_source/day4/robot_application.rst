Robot Application
=================

1. UR5 Simulation setup
-----------------------

1.1 Introduction
~~~~~~~~~~~~~~~~
This section describes how to setup UR5 robot simulation in Gazebo along with Kinect sensor.

1.2 Installation
~~~~~~~~~~~~~~~~
* ``gazebo_ros_pkgs`` (this package is already installed on your system)
* ``universal_robots`` (this package has Moveit configuration packages for different Universal robots)
  
  .. code-block:: bash

    $ cd ~/moveit_ws/src
    $ git clone https://github.com/ros-industrial/universal_robot
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash

1.3 Startup
~~~~~~~~~~~
Open each command below in a new terminal:

**Simulation**:

.. code-block:: bash

    $ roslaunch ur5_moveit_config_pkg ur5_gazebo.launch

(Press **play** in Gazebo GUI to start the simulation)

.. code-block:: bash

    $ roslaunch ur5_moveit_config_pkg ur5_moveit_planning_execution.launch sim:=true
    $ rviz

**Real robot**:

.. code-block:: bash

    $ roslaunch ur5_moveit_config_pkg ur5_bringup_joint_limited.launch robot_ip:=192.168.0.10
    $ roslaunch ur5_moveit_config_pkg ur5_moveit_planning_execution.launch sim:=false
    $ rviz

1.4 RViz configuration
~~~~~~~~~~~~~~~~~~~~~~
* Add the “Motion Planning” type into RViz
* Set the fixed frame to “world”

2. AR Alvar Tag Detection
-------------------------

2.1 Introduction
~~~~~~~~~~~~~~~~
Object recognition is essential in higher level robotic applications. To achieve this,
the goal in this tutorial is to detect the fiducial markers (“AR Tags”) based on RGB
data given by a simulated RGBD camera.

.. figure:: /_static/day4/app_1.png
   :width: 700px
   :align: center

   Simulation environment

2.1 Installation and Startup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
AR Markers are fiducial markers. They provide a way of visual pose estimation. 
AR Markers can be detected by the `ar_track_alvar` package. This package
can be installed in the following manner along with the package `openni2_launch` to
read camera images:

.. code-block:: bash

    $ sudo apt-get install ros-noetic-openni2-launch

In a new terminal, start the launch file ar.launch from the tutorial_commons package to detect the AR Tags
in the simulation as well. AR Track Alvar works out of the box by just running it, you
only need to determine the length of one side of the tag and the image topic to be
used. This is parameterized in the launch file.

.. code-block:: bash

    $ roslaunch tutorial_commons ar.launch 


.. code-block:: bash

    marker size: 13cm x 13cm
    camera image topic: /camera/rgb/image_raw
    camera info topic: /camera/rgb/camera_info
    output frame: /camera_link


When you start up AR-Track Alvar you can visualize the detected AR Tags by
enabling the topic /visualization_marker in Rviz. You can also visualize (and make use
of) tf data provided by AR-Track Alvar as it also represents detected Markers by
publishing transforms.
Once you started it up for the first time and set up the appropriate visualizations in
rviz you'll probably notice a screen like the following:

.. figure:: /_static/day4/app_2.png
   :width: 700px
   :align: center

   AR Tags at wrong size

3. Application: “Move arm above AR Tag”
---------------------------------------

3.1 Introduction
~~~~~~~~~~~~~~~~

This section describes the final task for this session. It is supposed to wrap up your
knowledge about URDF, TF and MoveIt! and create a real-world application with it.

3.2 Task description
~~~~~~~~~~~~~~~~~~~~
The final task for the application development is to complete a python node that
interacts with the UR5 MoveIt's moveit_commander interface to move the robot's end-
effector above the AR Tag. See figure 1 for further description.
The node of interest is called `arm_move.py` in the moveit_tutorial package. It contains
several methods to modify the position of the end-effector and to process data given
by the AR Track Alvar node.
Your task is to simply complete the node to let the arm move above the AR tag in
a continuous loop. Following is the list of tasks to be completed:

.. code-block:: python
    :linenos:

    #!/usr/bin/env python3

    import sys
    import rospy
    import moveit_commander
    import tf
    from geometry_msgs.msg import Pose
    from ar_track_alvar_msgs.msg import AlvarMarkers

    #declare global variables
    world_frame = "world"
    vel_scaling =  .3
    home_position = [.4, .0, .3]
    home_orientation = [.0, .0, .0, .1]
    marker_pose = None

    #convert marker from camera frame to robot's base frame
    def transform_pose(pose, target_frame):
        if tf_listener.canTransform(target_frame, pose.header.frame_id, rospy.Time(0)):
            #transform pose
            transform = tf_listener.transformPose(target_frame, pose)
            return transform.pose

    #callback function to receive marker messages
    def marker_cb(msg):
        global marker_pose
        if len(msg.markers) == 0:
            return
        marker = msg.markers[0]
        marker.pose.header.frame_id = marker.header.frame_id
        marker_pose = transform_pose(marker.pose, world_frame)

    #set Pose message through lists
    def set_pose(xyz = [0, 0, 0], q = [0, 0, 0, 1]):
        pose = Pose() 
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    #confirm if plan should be executed
    def plan_accepted():
        return input("Do you want to execute the plan [y] or replan [n]? ") == "y"

    #plan and execute to given pose; If plan is not confirmed plan again
    def plan_and_execute(group, pose):
        group.set_pose_target(pose)
        if plan_accepted():
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
        else:
            exit()

    #main function of application
    def main():
        #initialize moveit
        moveit_commander.roscpp_initialize(sys.argv)

        # Your Code For Task 1 #
        group.set_max_velocity_scaling_factor(vel_scaling)


        #while loop to move the robot to the found AR marker
        while not rospy.is_shutdown():
            plan_and_execute(group, set_pose(home_position, home_orientation))
            if marker_pose:
                marker = [marker_pose.position.x, marker_pose.position.y, home_position[2]]
                plan_and_execute(group, set_pose(marker))
            else:
                rospy.logwarn("No marker detected.")

    if __name__ == '__main__':
        rospy.init_node('move_to_marker', anonymous=True)
        tf_listener = tf.TransformListener()
        
        # Your Code for Task 2 # 

        main()


* **Task 1**: Instantiate ``MoveGroupCommander`` with the correct `group name`. 
  Refer to Rviz to get the group name in the `Motion Planning`` tab.

  .. code-block:: python

    group = moveit_commander.MoveGroupCommander("group name")

* **Task 2**: Add a subscriber to ar pose marker topic. Find out the message type of the topic from command line using rostopic.
  
  .. code-block:: bash

    $ rostopic info /ar_pose_marker
  
  .. code-block:: python

    rospy.Subscriber("topic name", AlvarMarkers, marker_cb)

* **Run** 
  
  .. code-block:: bash

    $ rosrun moveit_tutorial arm_move.py 

Now change the position of marker in the Gazebo simulation.