## Step up current project
```cd src```
```catkin_create_pkg robot_path_planner rospy std_msgs```



-------------------------------------------------------------------------------------
## Step
setup linux auto start ros 
    terminal: ```ls -a ~```
    terminal: ```nano .bashrc```
    add at last line: ```source /opt/ros/noetic/setup.bash```

## Step
This has to be ran at the backend:
    terminal: ```roscore```

## Create a workplace env
[tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

## Create a Package
[tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

## Functionalities
### rqt graph
    install:
        ```sudo apt-get install ros-<distro>-rqt```
        ```sudo apt-get install ros-<distro>-rqt-common-plugins```
    usage:
        ```rosrun rqt_graph rqt_graph```


### rostopic
    check available command:
        ```rostopic -h```
    
    publish commands:
        ```rostopic pub [topic] [msg_type] [args]```
        ```rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'```

### rqt plot
    GUI plot monitor for specific topic / variables:
        ```rosrun rqt_plot rqt_plot```
