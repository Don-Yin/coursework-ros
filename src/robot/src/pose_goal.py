#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np


def move_enc_effector():
    """
    end effector name: end_effector; needle
    end effecor belongs to: needle_group
    end effector parent component: sphere
    """

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_effector_node", anonymous=True)

    robot = moveit_commander.RobotCommander()

    # Replace with your group name
    group_name = "needle_group"
    group = moveit_commander.MoveGroupCommander(group_name)

    # Get the current pose
    current_pose = group.get_current_pose().pose

    # Generate a random offset within a range of -0.1 to +0.1
    random_offset = np.random.uniform(-0.1, 0.1, size=3)

    # Apply the offset to the current position
    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation = current_pose.orientation
    target_pose.position.x = current_pose.position.x + random_offset[0]
    target_pose.position.y = current_pose.position.y + random_offset[1]
    target_pose.position.z = current_pose.position.z + random_offset[2]

    group.set_pose_target(target_pose)

    # Plan and move
    plan = group.plan()
    group.execute(plan, wait=True)

    # Ensure the group stops
    group.stop()
    # Clear all targets after movement
    group.clear_pose_targets()


if __name__ == "__main__":
    try:
        move_enc_effector()
    except rospy.ROSInterruptException:
        pass
