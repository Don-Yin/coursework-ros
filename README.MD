

# Requirements
```pip install -r requirements.txt```


# End to End pipeline
```source pipeline.sh```
Note: this pipeline does not include the Visual Validation on the calculated trajectory, which has to be run separately as seen in below.


# Visual Validation on the calculated trajectory
1. plan the path: ```python 1_plan_path.py```
2. show volume: ```python 2_show_volume.py```
Note: this will ideally pops up three browser windows in terminal each showing the trajectory and one structure.

# Unit test
```python test.py```
Note: this is the unit test for the check_intersect_batch function which is the main function that selects out the trajectory that fits with the desired constraints. all details are written as doc strings and inline comments in the script.


# Video demo of the execution
link
update video




------

# Side Notes
Notes that are associated with the process of making the repository, notes on some intermediate code; not directly assocaited with the execution of the path planning.

## Create work space
The work space is created by ```catkin_make``` which makes:
- ```src``` and ```src/CMakeLists.txt```
- ```devel```
- ```build```

## Making Individual Packages
All packages should be made inside of the src folder by creating new directories using:
```catkin_create_pkg package_name dependencies```

## Ros Topics
show list:
```rostopic list```
echo data from a topic:
```rostopic echo topic_name```