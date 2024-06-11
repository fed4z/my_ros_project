# my_ros_project

Small demo to show how fast is to set up ros environent into docker and run some nodes inside it. In addition it is possible to test the comunication between ros nodes running in different containers.

# install & run
Install docker app, run the docker app.
(optional for development )Open vs code and install the remote development to be abel to connet vs code with the containers. 

Clone this repo in your host machine and its folder (my_ros_project):

```
cd my_ros_project
```

build the docker images
```
docker-compose build
```

```
docker-compose up -d
```

run the two containers with the following code:
```
docker-compose up -d
```

access to the first container
```
docker exec -it node1 bash
```

and in a new terminal
access to the second container
```
docker exec -it node2 bash
```

In the first container run:
```
colcon build && source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run my_package my_node
```

To check if the node is visible from other containers  in the second container run:

```
source /opt/ros/foxy/setup.bash && ros2 node list
```

and in the second conatiner should be displayed the running nodes.

To exit from a container:

```
exit
```

If you want to stop the containers, write in the host machine terminal:

```
 docker stop node1 node2
```







