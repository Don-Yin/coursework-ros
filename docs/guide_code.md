## Software and RoboticIntegration Final Report
Please provide a full-length report on ROS path planning software by June 2, 2023at 5pm. This system should comprise the following components:

## Repo
https://gitlab.com/rsparks/7mri0070

--------------------------------------------------------------------------------------------------------------------
## System Description
### Path Planning:
A implementation to select a straight trajectory from a set of inputs. This algorithm should take in a set of possible entry and target points (represented as a vtkMRMLMarkupsFiducialNode), and two binary image volumes representing the critical structures and target or structure (represented as either a vtkMRMLLabelMapVolumeNode or vtkMRMLLabelVolumeNode). It should return two points representing a selected final trajectory (represented as either a user defined type or a vtkMRMLMarkupsFiducialNode). The final trajectory should be selected with the constraints (a) avoidance of a critical structure, (b) placement of the tool into a target structure, (c) trajectory is below a certain length, (d) maximizing distance from critical structures (as in Coursework 1). Note this algorithm will be tested on the BrainPlanning dataset provided.

### ROS Robot Model:
define a RobotModel and appropriate helper classes so that its end effector can be moved to a given position (as in Coursework 2), determined from the data read from path planning. The RobotModel class should following the Unified Robot Description Format and a robot with at least two joints (6DOF are recommended for achieving the correct position). There should be a class that listens to data transmitted on an appropriate channel and identifies the point/pose requested and then tries to move the end effector of the RobotModel to the requested point/pose and prints an message if the robot cannot achieve the position.

## Validation
Validation should be performed to demonstrate the selected point visually and the robot end effector are co-located. This evaluation should be performed with ROS. Validation should be performed to demonstrate the selected point in 3D Slicer and the robot end effector are co-located.

## Note
May use moveit
--------------------------------------------------------------------------------------------------------------------

## Your task