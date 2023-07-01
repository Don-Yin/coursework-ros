#!/usr/bin/env python3

import rospy
import sys
import moveit_msgs
import moveit_commander
import geometry_msgs
import numpy as np


def pose_goal():
    # initial some node information so ROSCore knows what we are
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("pose_goal", anonymous=True)

    # start some moveit specific classes for the planning -- this is robot specific
    robot = moveit_commander.RobotCommander()
    group_name = "needle_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # We need to specify the end effector
    move_group.set_end_effector_link("needle")

    # Print out the current end effector position
    current_pose = move_group.get_current_pose().pose
    print("Current end effector position: ", current_pose)

    # Generate random values for each axis
    rand_x = current_pose.position.x + np.random.uniform(-0.05, 0.05)
    rand_y = current_pose.position.y + np.random.uniform(-0.05, 0.05)
    rand_z = current_pose.position.z + np.random.uniform(-0.05, 0.05)

    # set up a pose goal -- right now just hard coded
    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    pose_goal.position.x = rand_x
    pose_goal.position.y = rand_y
    pose_goal.position.z = rand_z

    # set the pose goal
    move_group.set_pose_target(pose_goal)

    # plan the motion and execute
    plan = move_group.go(wait=True)

    # stop the program from exiting until the motion is finished
    move_group.stop()
    move_group.clear_pose_targets()

    # shut down moveit_commander
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        pose_goal()
    except rospy.ROSInterruptException:
        pass
