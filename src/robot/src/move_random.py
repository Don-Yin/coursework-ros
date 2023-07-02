#!/usr/bin/env python3

import rospy
import sys
import moveit_commander


def move_end_effector():
    # initial some node information so ROSCore knows what we are
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_end_effector", anonymous=True)

    # start some moveit specific classes for the planning -- this is robot specific
    robot = moveit_commander.RobotCommander()
    group_name = "arm_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # We need to specify the end effector
    move_group.set_end_effector_link("needle")

    # Set random target position
    move_group.set_random_target()

    # plan the motion and execute
    plan_success = move_group.go(wait=True)

    # If the plan fails, alert the user and try again
    while not plan_success:
        print("Planning failed, trying again")
        move_group.set_random_target()
        plan_success = move_group.go(wait=True)

    # stop the program from exiting until the motion is finished
    move_group.stop()
    move_group.clear_pose_targets()

    # shut down moveit_commander
    moveit_commander.roscpp_shutdown()


def command_needle(pose: str):
    # initial some node information so ROSCore knows what we are
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_end_effector", anonymous=True)

    # start some moveit specific classes for the planning -- this is robot specific
    robot = moveit_commander.RobotCommander()
    group_name = "needle_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set the target to the predefined pose
    move_group.set_named_target(pose)

    # plan the motion and execute
    plan_success = move_group.go(wait=True)

    # If the plan fails, alert the user
    while not plan_success:
        print("Planning failed, trying again")
        move_group.set_named_target(pose)
        plan_success = move_group.go(wait=True)

    # stop the program from exiting until the motion is finished
    move_group.stop()
    move_group.clear_pose_targets()

    # shut down moveit_commander
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        move_end_effector()
        command_needle("Retracted")
    except rospy.ROSInterruptException:
        pass
