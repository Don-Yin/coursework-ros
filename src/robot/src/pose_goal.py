#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list


def move_end_effector():
    """
    end effector name: end_effector; needle
    end effecor belongs to: needle_group
    end effector parent component: sphere
    """

    # Initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_end_effector", anonymous=True)

    # Initialize MoveGroupCommander for the 'needle_group'.
    group = moveit_commander.MoveGroupCommander("needle_group")

    # Specify the target pose.
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = 0.5
    pose_target.position.y = 0.5
    pose_target.position.z = 0.5

    # Orientation in quaternion, setting to identity quaternion (no rotation).
    pose_target.orientation.w = 1.0

    # Set the target pose.
    group.set_pose_target(pose_target)

    # Plan the trajectory to the target pose.
    plan = group.plan()

    # Execute the planned trajectory.
    group.go(wait=True)

    # Clear targets after moving.
    group.clear_pose_targets()


if __name__ == "__main__":
    try:
        move_end_effector()
    except rospy.ROSInterruptException:
        pass
