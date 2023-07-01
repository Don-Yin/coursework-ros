#!/usr/bin/env python3

import rospy
import sys
import moveit_msgs
import moveit_commander
import geometry_msgs
import numpy as np
import time


def move_end_effector():
    # initialize the node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_end_effector", anonymous=True)

    # start MoveIt classes
    robot = moveit_commander.RobotCommander()
    group_name = "needle_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # specify the end effector
    move_group.set_end_effector_link("needle")

    # get and print the current end effector position
    current_pose = move_group.get_current_pose().pose
    print("Current end effector position:", current_pose)

    # start listening and printing every second
    while not rospy.is_shutdown():
        current_pose = move_group.get_current_pose().pose
        print("Current end effector position:", current_pose)
        time.sleep(1)

    # shutdown MoveIt
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        move_end_effector()
    except rospy.ROSInterruptException:
        pass
