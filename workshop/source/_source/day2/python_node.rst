ROS Python node
===============
This tutorial explains the basics of ROS Nodes, Launch files and the Parameter Server. You can use the given links in the documentation for further information. This tutorial is based on the ROS package maskor turtlesim, which is a modified version of the original turtlesim package provided by ROS.

    * Close all running processes before beginning a new chapter!

Writing a node in Python
------------------------ 
1.  Create a Python script file inside your package

.. prompt:: bash $
    
    roscd myfirstpackage/script/
    touch myfirstnode.py

Instead, you can also navigate to the package myfirstpackage using the graphical user interface and create a new empty document via right click.

1. Make the Python file executable

.. prompt:: bash $

    roscd myfirstpackage/script 
    chmod +x myfirstnode.py

Instead, you can also use the graphical user interface to open the script and/or use another text editor, e.g. Pluma or Nano or VS Code or any Programming IDE of your choice.
Given below is an example structure of a ROS node. It is the simplest way to code a ROS node in Python.

.. code-block:: python
    
    #!/usr/bin/env python3
    import rospy 
    
    def main(): 
        rospy.init_node(“myfirstnode”) 
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown(): 
            # ADD YOUR CODE HERE 
            hello_str = “hello world %s” % rospy.get_time() 
            rospy.loginfo(hello_str) 
            rate.sleep () 
    
    if __name__ == '__main__': 
        main()