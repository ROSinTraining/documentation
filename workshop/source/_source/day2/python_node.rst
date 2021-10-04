ROS Python node
===============
This tutorial explains the basics of ROS Nodes, Launch files and the Parameter Server. You can use the given links in the documentation for further information. This tutorial is based on the ROS package maskor turtlesim, which is a modified version of the original turtlesim package provided by ROS.

    * Close all running processes before beginning a new chapter!

Writing a node in Python
------------------------ 
1. Create a Python script file inside your package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash 
    
    $ roscd myfirstpackage/script/
    $ touch myfirstnode.py

Instead, you can also navigate to the package myfirstpackage using the graphical user interface and create a new empty document via right click.

2. Make the Python file executable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash 

    $ roscd myfirstpackage/script 
    $ hmod +x myfirstnode.py

Instead, you can also right click on the Python script: Properties → Permissions → 'Allow executing file as program'.

3. Program a ROS node
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash 

    $ roscd myfirstpackage/script/ 
    $ gedit myfirstnode.py

Instead, you can also use the graphical user interface to open the script and/or use another text editor, e.g. Pluma or Nano or VS Code or any Programming IDE of your choice.
Given below is an example structure of a ROS node. It is the simplest way to code a ROS node in Python.

.. code-block:: python
    :linenos:

    #!/usr/bin/env python3
    import rospy
    
    def main():
        rospy.init_node("myfirstnode")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #Add your code here
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            rate.sleep()
    
    if __name__ == '__main__':
        main()

The following lines are only explanations of the given code structure. You don't have to enter them in the terminal!

.. code-block:: python

    #!/usr/bin/env python3

The first line makes sure the script is executed as a Python script. Every Python ROS Node has to have this declaration at the top.

.. code-block:: python

    import rospy

Imports the Python library for ROS

.. code-block:: python

    rospy.init_node("myfirstnode")

This will initialize a ROS node with the name **myfirstnode**.

.. code-block:: python

    rate = rospy.Rate(10)

This line creates a Rate object rate. My With the help of this object and sleep(), it offers a convenient way for looping at the desired rate. 
With its argument of 10, we should expect to go through the loop 10 times per second, as long as our processing time does not exceed 1/10th of a second! 
http://wiki.ros.org/rospy/Overview/Time

.. code-block:: python

    while not rospy.is_shutdown():
        #Add your code here
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        rate.sleep()

This endless loop keeps running until the Node is shutdown. http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown

* Run the ROS node myfirstnode.
* Display information about the active nodes of the ROS system.

Launch Files
------------
A robot system in ROS consist of several interconnected nodes, which are responsible for different tasks, e.g.:

* Localization
* Locomotion
* Path planning
* Mapping
* Reading sensor data
* visualization
* etc...

Instead of invoking every node on its own, use a launch file! One launch file can start multiple nodes. 
It is a perfect tool for managing the processes of a more complex ROS application. 
On top a launch file can include other launch files. This makes it even easier to structure the complex starting process of a robot system. 
A launch file should be placed in a folder named launch inside the ROS package.

Create a launch file named turtle.launch:

.. code-block:: bash

    $ roscd myfirstpackage 
    $ mkdir launch 
    $ cd launch 
    $ touch turtle.launch

Instead, you can also create the launch folder and the launch file using the graphical user interface and right click.

Include multiple nodes in one launch file:

* Include the nodes maskor_turtlesim_node and maskor_turtle_teleop_key of the package maskor_turtlesim in the turtle.launch file. 
  The example given below explains the basic syntax of a launch file

.. code-block:: xml
    :linenos:
    
    <?xml version = "1.0"?> 
    <launch> 
        <node pkg = "mypackage" type = "publisher" name = "publisher" /> 
        <node pkg = "mypackage" type = "subscriber" name = "subscriber" /> 
    </launch>

A launch file follows the standard XML syntax:
* name -> Name for the running process
* pkg -> Name of the package that holds the executable
* type -> Name of the executable file
* output -> Location of the output ("screen" or "log")

Every launch file must consist the tag <launch> at the beginning and </launch> at the end. 

Start a launch file: ``roslaunch myfirstpackage turtle.launch``

.. code-block:: bash

    $ roslaunch myfirstpackage turtle.launch

The roslaunch command parses a launch file in XML format. Include a launch file in a launch file:

.. code-block:: xml
    :linenos:
    
    <?xml version ="1.0"?>
    <launch >
        <include file ="$(find some_package)/launch/launchfile.launch" /> 
    </launch>

* Start the launch file.
* Include your node myfirstnode into the launch file, restart it and check all active nodes of your system. 
* For further information: http://wiki.ros.org/roslaunch

Parameter Server
----------------

The parameter server stores and retrieves parameters of ROS nodes at runtime. It is suitable for static data. The parameter server is accessible via:

* command line
* launch file or
* ROS node.

Start the ROS node maskor turtlesim node of the package maskor turtlesim.

Access via commandline
~~~~~~~~~~~~~~~~~~~~~~

List all active parameters within the ROS Parameter Server:

.. code-block:: bash

    $ rosparam list

By executing this command, a list of the current parameters on the parameter server is dis- played. The parameters background_r, background_g and background_b define the back- ground color. The former part is a namespace and depends on the name of the running process, which is /turtlesim per default.

.. hint:: You can change the default name of a running process using a launch file.

Get the actual value of a specific parameter: ``rosparam get <param_name>``

.. code-block:: bash

    $ rosparam get /background_b

Displays the actual value of the parameter. Store a value to the ROS Parameter Server: ``rosparam set <param_name> <value>``

.. code-block:: bash

    $ rosparam set /background_b 100