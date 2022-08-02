TF TUTORIAL
===========

1. Introduction
-----------------
This tutorial will show you the benefits and the power of the ROS TF tool. It will introduce how to make use of static and dynamic transforms and TF listener. 

1. Writing a tf broadcaster
----------------------------
First of all create a new package in your workspace. Name it ``learning_tf`` with dependencies on **tf2**, **tf2_ros**, **tf_conversions**, **rospy** and 
**turtlesim** and build the package. Inside of the package create a new python node called ``tf_broadcaster_example.py`` *(~/moveit_ws/src/learning_tf/scripts/)*.

.. code-block:: bash

    $ catkin_create_pkg learning_tf rospy roscpp tf tf2 tf2_ros geometry_msgs turtlesim

Given below is an example of a ROS node including a TF Broadcaster. The code is based on the simple structure of the node. Feel free to adjust the code structure based on your programming skills. Copy the code to the ``tf_broadcaster_example.py``. 
Make the python file executable.

.. code-block:: python
    :linenos:

    #!/usr/bin/env python3
    import rospy
    # Because of transformations
    import tf_conversions
    import tf2_ros
    import geometry_msgs.msg
    import turtlesim.msg
    def handle_turtle_pose(msg, turtle_name):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = turtle_name
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)

    if __name__ == '__main__':
        rospy.init_node('tf2_turtle_broadcaster')
        turtle_name = rospy.get_param('~turtle')
        rospy.Subscriber('/%s/pose' % turtle_name, turtlesim.msg.Pose, handle_turtle_pose, turtle_name)
        rospy.spin()

2.1 Explanation:
~~~~~~~~~~~~~~~~
The basic structure of a simple node including Publisher and Subscriber is already well known. So, only the tf specific lines will be explained.

.. code-block:: python

    import tf_conversions
    import tf2_ros

Import the essential TF modules. The ``tf2_ros`` package provides ROS bindings to tf2. ``tf_conversions`` provides the popular ``transformations.py``, 
which was included in tf but not in tf2, in order to have a cleaner package.

.. code-block:: python

    br = tf2_ros.TransformBroadcaster()

This will initialize a ROS TF Broadcaster, which allows to send transforms from one frame to another one.

.. code-block:: python

    turtle_name = rospy.get_param("~turtle")

This line gets the value of the ``turtle`` parameter from the parameter server (e.g. ``turtle1``). The ~ prefix prepends the parameter-name with 
the node's name to use it as a semi-private namespace (e.g. ``/tf_broadcaster/turtle``).

.. code-block:: python

    br.sendTransform(t)

This line is the handler function to provide the transform between the ``turtle1`` (value of the `turtle` parameter) and the ``world`` frame and publishes it. 
As it is inside of the while loop and the rate is set to 30 Hz, the transform will be published within this frequency. The `sendTransform` function a 
``StampedTransform``, which was set upin the subscriber callback:

* **Header**
  
  * *Timestamp* (Determine  the  moment  when  this  transform  is  happening. This is mainly `rospy.Time.now()` when you want to send the actual transform. This  means  the  transform  can  change  over  time  to  generate  a  dynamic motion.)
  * *Frame_ID* (The frame ID of the OriginFrame)
* **Child Frame ID** (Frame ID to which the transform ishappening)
* **Transform** 
  
  * *Position* in m (X, Y andZ)
  * *Orientation* in Quaternion (You can use the TF Quaternion from Euler function to use the roll, pitch and yaw angles in radinstead)

2.2 Testing theBroadcaster
~~~~~~~~~~~~~~~~~~~~~~~~~~
Create  a  launch  file  called  ``tf_examples.launch``  in  your  package.  Include  the turtlesim_node and the turtle_teleop_keynode from the turtlesim package. Include the example Broadcaster node in the launch file with private parameter 
“turtle” of type string. Set the value of “turtle” as “turtle1”, and run the launch file.

.. code-block:: xml
    :linenos:

    <launch>
        <!-- Turtlesim Node-->
        <node pkg="turtlesim" type="turtlesim_node" name="sim"/>
        <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>

        <node name="turtle1_tf_broadcaster" pkg="learning_tf" type="tf_broadcaster_example.py" respawn="false" output="screen" >
        <param name="turtle" type="string" value="turtle1" />
        </node>
    </launch>

Now, use the ``tf_echo`` tool to check if the turtle pose is already published to tf:

.. code-block:: bash

    $ rosrun tf tf_echo /world/turtle1

This should show you the pose of the turtle1 related to the world frame. Now, drive around  the  turtle  using  the  arrow  keys.  
Make  sure  to  have  the  terminal  in foreground that started the launch file including the keyboard teleopnode.You can use ``rqt_tf_tree`` 
to check available TF trees.

.. code-block:: bash

    $  rosrun rqt_tf_treerqt_tf_tree

``rqt_tf_tree`` is  a  runtime  tool  for  visualizing  the  tree  of  frames  being  broadcasted  via ROS. You can refresh the tree simply by the refresh 
button in the top-left corner of the GUI.Also rviz can be used to visualize the location of frames. Start rviz.


.. code-block:: bash

    $ rosrun rviz rviz

Add  a  TF  visualization  element  in  rviz  and  set  the  fixed  frame  to  world.  Move  the turtle around and follow the location of the turtle1 frame in world.

1. Writing a TFlistener
-----------------------

TF provides much more tools then just the Broadcaster. A couple of debugging and visualization  tools  for  frames  have  been  introduced  recently.  Also  
very  powerful  is the access to frame transformations. This can be done using TF listener. TF listener solvesinverse  kinematics.  Similar  to the  command  
line tool tf_echo,  TF  listener  can check the transformation between two frames in nodes. In the following example we will  add  a  second  turtle  to  
the  turtlesimnode.  The  second  turtle  should  follow  the first one. Add the following example code to a python node  ``tf_listener_example.py`` 
inside of the ``learning_tf`` package. Make the python file executable.

.. code-block:: python
    :linenos:

    #!/usr/bin/env python3  
    import rospy

    import math
    import tf2_ros
    import geometry_msgs.msg
    import turtlesim.srv

    if __name__ == '__main__':
        rospy.init_node('tf2_turtle_listener')

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        turtle_name = rospy.get_param('turtle', 'turtle2')
        spawner(4, 2, 0, turtle_name)

        turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            msg = geometry_msgs.msg.Twist()

            msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

            turtle_vel.publish(msg)

            rate.sleep()

3.1 Explanation
~~~~~~~~~~~~~~~

.. code-block:: python

    import tf2_ros

Importing tf is necessary to use the tf listener functionalities.

.. code-block:: python

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

A listener has to be initialized first. It utilizes a buffer to keep track of past transforms.

.. code-block:: python

    try:
        trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

The listener has to be used inside a ``try`` and ``except`` block. The listener itself.

.. code-block:: python

    trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())

lookups the transform from “turtle2” to “turtle1” in the actual moment (``rospy.Time(0)``) and  stores  the  result  
in  the  Transform  variable  trans  which  holds  Orientation  and Position.

.. code-block:: python

    msg = geometry_msgs.msg.Twist()

    msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
    msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

    turtle_vel.publish(msg)

The publisher afterwards will make the second turtle move and let it follow the first one with the corresponding mathematics.

3.2 Testing the Listener
~~~~~~~~~~~~~~~~~~~~~~~~
Include the listener node to the previous generated launch file ``tf_examples.launch`` and start it. 

.. code-block:: xml

    <node pkg="learning_tf" type="tf_listener_example.py"  name="listener" output="screen"/>


The lookup will fail for now as we did not spawn the second turtle right now. To do so, a ROS Service call will be used. We will also need to launch a second 
TransformBroadcaster:

.. code-block:: bash

    $ rosservice call /spawn 2 2 0.2 "turtle2"
    $ rosrun learning_tf tf_listener_example.py _turtle:=turtle2

This  will spawn  the  second  turtle  with  the  initial  position  x  =  2  and  y  =  2,  an orientation of yaw  =  0.2  rad  and  the name “turtle2”.  
Move  now  the first turtle around and the second turtle should start to follow the firstone.

.. hint:: 

    Try to spawn ``turtle2`` inside ``tf_examples.launch`` file.

    .. code-block:: xml

        <node name="turtle2_tf_broadcaster" pkg="learning_tf" type="tf_broadcaster_example.py" respawn="false" output="screen" >
            <param name="turtle" type="string" value="turtle2" />
        </node>

4. Adding static transforms
---------------------------
Another node that is provided by the ROS TF tool is the static_transform_publisher. It  can  be  used  to  determine  static  frame  transforms,  
e.g.  from  a  robot  base  to  a sensor devices frame. This one is quite easy to use. Add a virtual camera frame to our  first  turtle  within  the  
recent  launch  file  ``tf_examples.launch``  by  adding  the following line:

.. code-block:: xml

    <node pkg="tf2_ros" type="static_transform_publisher" name="turtle1_cam_frame" args="0.1 0.0 0.0 -1.57 0.0 0.0  turtle1 turtle_cam" />

This will add the frame “turtle_cam” with respect to to the “turtle1” frame. The virtual  camera  is  mounted  +0.1  m  in  x-axis  from  view  of  the  turtle  base  and rotated -1.57  rad  in yaw. 
The  convention  for  the  static  transform  publisher arguments is  the following order: X  Y  Z  Yaw  Pitch  Roll  Parent_frame  Child_frame publisher_framerate.

4.1 Testing the Listener
~~~~~~~~~~~~~~~~~~~~~~~~
To verify the location of the added virtual turtle camera, start the generated launch file,  use  rviz  add  the  TF  visualization  element.  
Set  fixed  frame  to  world  and move the first turtle around. You should see now both frames the turtle1 and the ``turtle1_cam_frame`` moving in the world.