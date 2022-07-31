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