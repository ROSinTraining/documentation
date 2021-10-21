TF TUTORIAL
===========

1. Introduction
-----------------
This tutorial will show you the benefits and the power of the ROS TF tool. It will introduce how to make use of static and dynamic transforms and TF listener.

2. Writing a tf broadcaster
----------------------------
First of all create a new package in your workspace. Name it ``learning_tf`` with dependencies on **tf2**, **tf2_ros**, **tf_conversions**, **rospy** and 
**turtlesim** and build the package. Inside of the package create a new python node called ``tf_broadcaster_example.py`` *(~/moveit_ws/src/learning_tf/scripts/)*.

Given below is an example of a ROS node including a TF Broadcaster. The code is based on the simple structure of the node. Feel free to adjust the code structure based on your programming skills. Copy the code to the ``tf_broadcaster_example.py``. 
Make the python file executable.

.. code-block:: python
    :linenos:

    #!/usr/bin/env python  
    import rospy

    # Because of transformations
    import tf_conversions

    import tf2_ros
    import geometry_msgs.msg
    import turtlesim.msg


    def handle_turtle_pose(msg, turtlename):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = turtlename
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
        turtlename = rospy.get_param('~turtle')
        rospy.Subscriber('/%s/pose' % turtlename,
                    turtlesim.msg.Pose,
                    handle_turtle_pose,
                    turtlename)
        rospy.spin()