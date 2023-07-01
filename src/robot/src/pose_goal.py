#!/usr/bin/env python3

import rospy
import sys
import moveit_msgs
import moveit_commander
import geometry_msgs


def monitor_end_effector():
    # initial some node information so ROSCore knows what we are
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("monitor_end_effector", anonymous=True)

    # start some moveit specific classes for the planning -- this is robot specific
    robot = moveit_commander.RobotCommander()
    group_name = "needle_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # We need to specify the end effector
    move_group.set_end_effector_link("needle")

    # control the rate of printing
    rate = rospy.Rate(1)  # 1 Hz

    # main loop, print out the current end effector position every second
    while not rospy.is_shutdown():
        current_pose = move_group.get_current_pose().pose
        print("Current end effector position: ", current_pose)
        rate.sleep()

    # shut down moveit_commander
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        monitor_end_effector()
    except rospy.ROSInterruptException:
        pass
