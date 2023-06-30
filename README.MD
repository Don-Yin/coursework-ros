# Note
This repository works as a ros work place and the repository for all other python functions.

# Initial work space
The work space is created by ```catkin_make``` which makes:
- ```src``` and ```src/CMakeLists.txt```
- ```devel```
- ```build```

# Making individual packages
All packages should be made inside of the src folder by creating new directories using:
```catkin_create_pkg package_name dependencies```

# Running individual scripts in package
while running ros core / chmod +x to make the script itself executable:
```rosrun module_name script_name```

# Running launch files in package
this is a way of running multiple nodes at a same time:
while running ros core:
```roslaunch package_name launch_file_name```

# rostopics
show list:
```rostopic list```
echo data from a topic:
```rostopic echo topic_name```