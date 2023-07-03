#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import geometry_msgs
import numpy as np
from tf.transformations import quaternion_from_euler
import math


def convert_slicer_to_ros(point):
    scale_factor = 0.08
    point = point * scale_factor
    position_correction = np.array([-29.5, -10.5, 0])
    point = point + position_correction
    return np.array([-point[0], point[1], point[2]])


class CommandArm:
    def __init__(self):
        """
        The robot variable can be used for:
        1. Get the Robot's Kinematic Model
        2. Get the Current State of the Robot
        3. Check Joint Limits
        4. Get the Robot's End Effectors
        5. Set the Robot's Start State
        6. Check Collision
        """
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_end_effector", anonymous=True)
        self.robot = moveit_commander.RobotCommander()

    def end_effector_positon(self, coordinates: tuple):
        """orientation is not considered"""
        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("sphere")
        move_group.set_planning_time(10)

        move_group.set_position_target(coordinates)
        plan_success = move_group.go(wait=True)

        # while not plan_success:
        if not plan_success:
            print("Planning failed, trying again")

        move_group.stop()
        move_group.clear_pose_targets()

    def end_effector_orientation(self, entry: np.array, target: np.array):
        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("sphere")
        move_group.set_planning_time(10)

        # Calculate the direction vector from entry to target
        direction = target - entry
        direction = direction / np.linalg.norm(direction)

        # Assuming the 'up' direction is along z-axis of the robot base frame
        up = np.array([0, 0, 1])
        right = np.cross(direction, up)
        up = np.cross(right, direction)

        # Calculate roll, pitch, yaw angles from the direction and up vectors
        roll = np.arctan2(-up[1], up[0])
        pitch = np.arcsin(up[2])
        yaw = np.arctan2(-direction[1], direction[0])

        # Clear previous pose targets
        move_group.clear_pose_targets()
        move_group.set_rpy_target([roll, pitch, yaw])
        plan_success = move_group.go(wait=True)

        if not plan_success:
            print("Planning failed, trying again")

        move_group.stop()
        move_group.clear_pose_targets()

    def end_effector_position_orientation(self, entry: np.array, target: np.array):
        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("sphere")
        move_group.set_planner_id("LBTRRT")
        move_group.set_max_velocity_scaling_factor(1.0)
        move_group.set_max_acceleration_scaling_factor(1.0)

        move_group.set_goal_position_tolerance(3)
        # move_group.set_goal_orientation_tolerance(0.1)

        # Calculate the direction vector from entry to target
        direction = target - entry
        direction = direction / np.linalg.norm(direction)

        # Assuming the 'up' direction is along z-axis of the robot base frame
        up = np.array([0, 0, 1])
        right = np.cross(direction, up)
        up = np.cross(right, direction)

        # Calculate roll, pitch, yaw angles from the direction and up vectors
        roll = np.arctan2(-up[1], up[0])
        pitch = np.arcsin(up[2])
        yaw = np.arctan2(-direction[1], direction[0])

        # convert the roll, pitch, yaw to a quaternion
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        # Set the pose target
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = entry[0]
        pose_target.position.y = entry[1]
        pose_target.position.z = entry[2]
        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]

        move_group.set_pose_target(pose_target)

        max_attempts = 3
        num_attempts = 0
        plan_success = False
        move_group.set_planning_time(45)

        while not plan_success and num_attempts < max_attempts:
            print("Planning attempt: ", num_attempts)
            plan_success = move_group.go(wait=True)
            num_attempts += 1

        # if proceeded here and suceeded with some residuals on the orientation
        # try to correct the orientation by running the runtion again

        if not plan_success:
            print("Planning failed, trying again")

        move_group.stop()
        move_group.clear_pose_targets()

    # random poses ---------------------------------------------

    def move_random(self):
        """For testing purpose"""
        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("needle")
        move_group.set_random_target()
        plan_success = move_group.go(wait=True)

        while not plan_success:
            print("Planning failed, trying again")
            move_group.set_random_target()
            plan_success = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

    # default poses ---------------------------------------------

    def pose_needle(self, pose: str):
        robot = moveit_commander.RobotCommander()
        group_name = "needle_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        move_group.set_named_target(pose)
        plan_success = move_group.go(wait=True)

        while not plan_success:
            print("Planning failed, trying again")
            move_group.set_named_target(pose)
            plan_success = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

    def pose_arm(self, pose: str = "Home"):
        robot = moveit_commander.RobotCommander()
        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_max_velocity_scaling_factor(1.0)
        move_group.set_max_acceleration_scaling_factor(1.0)

        move_group.set_named_target(pose)
        plan_success = move_group.go(wait=True)

        while not plan_success:
            print("Planning failed, trying again")
            move_group.set_named_target(pose)
            plan_success = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

    def rotate_base(self):
        move_group = moveit_commander.MoveGroupCommander("arm_group")

        # Getting the current joint values for the robot:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = math.radians(-90)  # 'rotator1' joint
        move_group.go(joint_goal, wait=True)
        move_group.stop()

    def on_finish(self):
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    import json
    from pathlib import Path

    with open(Path("assets", "entry_target_real.json"), "r") as loader:
        data = json.load(loader)

    entry, target = data["original_coordinates"][0], data["original_coordinates"][1]
    entry, target = np.array(entry), np.array(target)

    entry = convert_slicer_to_ros(entry)
    target = convert_slicer_to_ros(target)

    try:
        command_arm = CommandArm()
        # command_arm.move_random()

        # command_arm.pose_arm("Home")
        # command_arm.rotate_base()
        # command_arm.pose_needle("Retracted")

        # command_arm.pose_needle("Extended")
        # command_arm.move_end_effector((10, 10, 10))
        # command_arm.end_effector_positon(entry)
        command_arm.end_effector_position_orientation(entry, target)
        command_arm.on_finish()
    except rospy.ROSInterruptException:
        pass
