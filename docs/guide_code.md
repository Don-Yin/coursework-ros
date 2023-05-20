### ROS Robot Model For Moving End Effector Along a Trajectory:
Define a RobotModel and appropriate helper classes so that its end effector can be moved to a given position, determined from the data read from path planning. The RobotModel class should following the Unified Robot Description Format and a robot with at least two joints (6DOF are recommended for achieving the correct position). There should be a class that listens to data transmitted on an appropriate channel and identifies the point/pose requested and then tries to move the end effector of the RobotModel to the requested point/pose and prints an message if the robot cannot achieve the position.

## Validation
Validation should be performed to demonstrate the selected point visually and the robot end effector are co-located. This evaluation should be performed with ROS. Validation should be performed to demonstrate the selected point and the robot end effector are co-located.

## Note
May use moveit package

## What we have at the current stage
- We have a virtual machine ready that has ROS (ROS1) installed
- The current repository is in this virtual machine.
- We have the robot model defined using the Unified Robot Description Format at path Path("assets", "arm.urdf")
- We have the calculated planed path ready, defined using json file at Path("asserts", "entry_target_real.json"), this file contains a dictionary a key of "original_coordinates" that points to a list where the first element is the entry coordinates (x, y, z) and the second element the target (x, y, z). i.e., entry, target = json_dict["original_coordinates"]
- We have the brain volume data in the format of nii.gz; I want you to visualize three types of volumes while making the algorithm, namely target_structure, critical_structure and cortex. Their file paths are:

  files_dict_real = {
      "target_structure": [
          Path("data", "BrainParcellation", "r_hippo.nii.gz"),
      ],
      "critical_structure": [
          Path("data", "BrainParcellation", "ventricles.nii.gz"),
          Path("data", "BrainParcellation", "vessels.nii.gz"),
      ],
      "cortex": [
          Path("data", "BrainParcellation", "cortex.nii.gz"),
      ],
  }
- We have a new catkin package in the in the src folder, namely src/move_robot, this package folder contains a CMakeLists.txt and a package.xml, both are at the default state.

## Note
- Avoid using 3D slicer 
- As much as possible, use Python instead of inputting commands in the linux terminal; only use the linux terminal for things that Python cannot do, such as starting roscore.
- In my situation, I want the end effector to start at a specific point - entry (x, y, z) - and stop at another place - target (x, y, z) - while keeping the arm parallel to the path (the trajectory between entry and target)

## High-level overview of the steps:
1. **Create a ROS Package**: Create a new ROS package to encapsulate your work.

2. **Set Up Robot Model in MoveIt**: Import your robot model (URDF file) into MoveIt. MoveIt provides functionalities for robot motion planning, including inverse kinematics, collision detection, and trajectory generation.

3. **Implement Inverse Kinematics**: Inverse kinematics is crucial for determining the joint parameters that provide a desired position for the robot's end effector. As part of the MoveIt setup, you would define the kinematics solver for your robot model. MoveIt supports various kinematics solvers like KDL, IKFast, and others. This step is implicit in the setup of MoveIt but is a crucial part of the process, as it enables MoveIt to calculate the robot's joint values to reach the target pose.

4. **Implement a Pose Listener Node**: This is a ROS node that subscribes to a topic where the path planning data will be published. This node will parse the incoming data and extract the required target pose.

5. **Move Robot to Target Pose**: Once the Pose Listener node has the target pose, it will use MoveIt's Python interface to plan and execute a movement of the robot's end effector to the target pose. This process involves using inverse kinematics to calculate the necessary joint configurations for the robot to reach the desired position.

6. **Handle Failure Cases**: If the robot cannot achieve the target pose, the node should print a message indicating this. This can be implemented in Python.

7. **Visualize the Robot and Brain Volumes**: For visualization, use RViz. Load the brain volumes into RViz as MarkerArray messages. To display the robot and the brain volumes in the same RViz window, make sure that their frames of reference align or can be transformed into a common frame. Consider using the `tf2_ros` package to manage the coordinate transformations between different frames.


## Context
The above are some descriptions of a path plan and ROS algorithm and an high level overview of each step.

# Your task
At the current stage, I want you to focus on this following task, I want you to provide a comprehensive guide for this specific task only:

2. **Set Up Robot Model in MoveIt**: Import your robot model (URDF file) into MoveIt. MoveIt provides functionalities for robot motion planning, including inverse kinematics, collision detection, and trajectory generation.