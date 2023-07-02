#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import geometry_msgs


class CommandArm:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_end_effector", anonymous=True)

    def move_end_effector(self, coordinates: tuple, orientations: tuple = None):
        if not self.check_reachability(coordinates):
            print("Position is not reachable")
            return

        robot = moveit_commander.RobotCommander()
        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("needle")

        # Set the planning time to a higher value
        move_group.set_planning_time(30)

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "base_link"  # Here you set the frame_id
        pose_goal.pose.position.x = coordinates[0]
        pose_goal.pose.position.y = coordinates[1]
        pose_goal.pose.position.z = coordinates[2]

        if orientations is not None:
            pose_goal.pose.orientation.x = orientations[0]
            pose_goal.pose.orientation.y = orientations[1]
            pose_goal.pose.orientation.z = orientations[2]
            pose_goal.pose.orientation.w = orientations[3]

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)

        while not plan:
            print("Planning failed, trying again")
            move_group.set_pose_target(pose_goal)
            plan = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()

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

    def check_reachability(self, coordinates: tuple):
        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base_link"
        pose_target.pose.position.x = coordinates[0]
        pose_target.pose.position.y = coordinates[1]
        pose_target.pose.position.z = coordinates[2]

        # Try to solve IK for the target pose
        joint_values = move_group.get_joint_value_target(pose_target)
        if joint_values is not None:
            return True  # Position is reachable
        else:
            return False  # Position is not reachable

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
    try:
        command_arm = CommandArm()
        # command_arm.move_random()
        # command_arm.pose_arm("Home")
        # command_arm.pose_needle("Retracted")
        # command_arm.pose_needle("Extended")
        command_arm.move_end_effector((10, 10, 10))
        command_arm.on_finish()
    except rospy.ROSInterruptException:
        pass
