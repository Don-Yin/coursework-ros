#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import geometry_msgs
import numpy as np
from tf.transformations import quaternion_from_euler


def convert_slicer_to_ros(point):
    scale_factor = 0.08
    point = point * scale_factor
    position_correction = np.array([-25.0, -10.0, 5.0])
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

        # Set the planning time to a higher value
        move_group.set_planning_time(10)
        move_group.set_position_target(coordinates)
        plan_success = move_group.go(wait=True)

        while not plan_success:
            print("Planning failed, trying again")
            move_group.set_position_target(coordinates)
            plan_success = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

    def rotate_end_effector(self, roll, pitch, yaw):
        """Rotate the end effector to desired euler angles while maintaining position"""
        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("sphere")

        # Get the current position
        current_pose = move_group.get_current_pose().pose.position
        x, y, z = current_pose.x, current_pose.y, current_pose.z

        # Convert euler angles (roll, pitch, yaw) to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)

        # Prepare the target pose
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]

        # Set the target pose
        move_group.set_pose_target(pose_target)
        plan_success = move_group.go(wait=True)

        # while not plan_success:
        if not plan_success:
            print("Planning failed")
            move_group.stop()
            move_group.clear_pose_targets()

        move_group.stop()
        move_group.clear_pose_targets()

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

        move_group.set_named_target(pose)
        plan_success = move_group.go(wait=True)

        while not plan_success:
            print("Planning failed, trying again")
            move_group.set_named_target(pose)
            plan_success = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

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
        # command_arm.pose_needle("Retracted")
        # command_arm.pose_needle("Extended")
        # command_arm.move_end_effector((10, 10, 10))
        command_arm.end_effector_positon(entry)
        command_arm.rotate_end_effector(np.radians(2), np.radians(2), np.radians(2))
        command_arm.on_finish()
    except rospy.ROSInterruptException:
        pass
