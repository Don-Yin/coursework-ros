#!/usr/bin/env python3

import rospy
import sys
import moveit_commander

import os
from pathlib import Path
import sys
sys.path.insert(0, 'src/modules')
# from src.modules.fcsv import FcsvParser
from fcsv import FcsvParser


class CommandArm:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_end_effector", anonymous=True)

    def move_random(self):
        """
        The robot variable can be used for:
        1. Get the Robot's Kinematic Model
        2. Get the Current State of the Robot
        3. Check Joint Limits
        4. Get the Robot's End Effectors
        5. Set the Robot's Start State
        6. Check Collision
        """
        robot = moveit_commander.RobotCommander()
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

    def command_needle(self, pose: str):
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

    def command_arm(self, pose: str = "Home"):
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


entires_fcsv = FcsvParser(Path("data", "entries.fcsv"))
targets_fcsv = FcsvParser(Path("data", "targets.fcsv"))

entries_coords = entires_fcsv.content_dataframe[["x", "y", "z"]].values
targets_coords = targets_fcsv.content_dataframe[["x", "y", "z"]].values

print(entries_coords)
print(targets_coords)


if __name__ == "__main__":
    try:
        pass
        # command_arm = CommandArm()
        # command_arm.move_random()
        # command_arm.command_arm("Home")
        # command_arm.command_needle("Retracted")
        # command_arm.command_needle("Extended")
        # command_arm.on_finish()
    except rospy.ROSInterruptException:
        pass
