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
    group_name = "needle_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # We need to specify the end effector
    move_group.set_end_effector_link("needle")

    # Print out the current end effector position
    current_pose = move_group.get_current_pose()
    print("Current end effector position: ", current_pose.pose)

    aim_x = 208.596 / 10  # 0.208596
    aim_y = 133.917 / 10  # 0.133917
    aim_z = 115.449 / 10  # 0.115449

    # set up a pose goal -- right now just hard coded
    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    pose_goal.position.x = aim_x
    pose_goal.position.y = aim_y
    pose_goal.position.z = aim_z

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
