Docker
======

Docker Basic Commands
---------------------
1. Hello world

    .. code-block:: bash
        
        $ docker run hello-world

2. Manage images
   
    .. code-block:: bash
        
        $ docker image <options>

3. List images 
   
    .. code-block:: bash
        
        $ docker images

4. Build an image from a Dockerfile
   
    .. code-block:: bash
        
        $ docker build <option> <Dockerfile>

5. Run a container
   
    .. code-block:: bash
        
        $ docker run <image>

6. List container
   
    .. code-block:: bash
        
        $ docker ps

7. Start one or more stoped container
   
    .. code-block:: bash
        
        $ docker start <options> <container> <container ..>

8.  Run a command in a running container
   
    .. code-block:: bash
        
        $ docker exec <option> <container ID> <command>

10. Stop one or more containers
    
    .. code-block:: bash
        
        $ docker stop <options> <containers>

11. Kill one or more containers

    .. code-block:: bash
        
        $ docker kill <containers ID>

Writing first Dockerfile
------------------------

Create necessary files.

.. code-block:: bash
    
    $ mkdir my_first_docker && cd my_first_docker
    $ touch Dockerfile
    $ touch docker-compose.yml
    $ touch ros_entrypoint.sh
    $ chmod +x ros_entrypoint.sh

1. Edit ``Dockerfile``.

.. code-block:: dockerfile
    :linenos:

    ARG ROS_DISTRO=melodic
    FROM ros:${ROS_DISTRO}-robot

    ENV DEBIAN_FRONTEND noninteractive

    RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-perception-pcl \
        && rm -rf /var/lib/apt/lists/*

    # setup entrypoint
    ENV ROS_DISTRO ${ROS_DISTRO}
    COPY ./ros_entrypoint.sh /
    RUN chmod a+rwx ros_entrypoint.sh

    ENTRYPOINT ["/ros_entrypoint.sh"]
    CMD ["bash"]

2. Edit ``docker-compose.yml``.

.. code-block:: yaml
    :linenos:

    version: '3'

    services:
        ros-melodic-base:
            build:
                context: .
                args:
                    - ROS_DISTRO=melodic
            image: ros-melodic-base
            stdin_open: true
            tty: true
            privileged: true
            network_mode: "host"
            volumes:
                - /tmp/.X11-unix:/tmp/.X11-unix:ro
                - /dev/shm:/dev/shm
                - /dev/*:/dev/*
            environment:
                - DISPLAY=$DISPLAY
            entrypoint: /usr/local/bin/scripts/workspace-entrypoint.sh
            command: bash -c "top"

3. Edit ``ros_entrypoint.sh``

.. code-block:: bash
    :linenos:

    #!/bin/bash
    set -e
    # setup ros2 environment
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    exec "$@"

4. Docker build.

.. code-block:: bash

    $ docker-compose up --build