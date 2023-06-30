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