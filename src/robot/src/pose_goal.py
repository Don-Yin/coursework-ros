#!/usr/bin/env python3

import rospy
import sys
import moveit_msgs
import moveit_commander
import geometry_msgs


def pose_goal():
    # initial some node information so ROSCore knows what we are
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("pose_goal", anonymous=True)

    # start some moveit specific classes for the planning -- this is robot specific
    robot = moveit_commander.RobotCommander()
    group_name = "AllJoints"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # set up a pose goal -- right now just hard coded
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    # solve for the pose goal
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    # clear out and residual information
    move_group.stop()
    move_group.clear_pose_targets()


if __name__ == "__main__":
    try:
        pose_goal()
    except rospy.ROSInterruptException:
        pass
