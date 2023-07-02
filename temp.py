
    def end_effector_position_orientation(self, entry: np.array, target: np.array):
        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("sphere")

        # Calculate the direction vector from entry to target
        direction = target - entry
        direction = direction / np.linalg.norm(
            direction
        )  # Normalize to get unit vector

        # Assuming the 'up' direction is along z-axis of the robot base frame
        up = np.array([0, 0, 1])

        # Calculate the right vector as the cross product of direction and up
        right = np.cross(direction, up)

        # Recalculate the up vector as the cross product of right and direction
        up = np.cross(right, direction)

        # Calculate roll, pitch, yaw angles from the direction and up vectors
        roll = np.arctan2(-up[1], up[0])
        pitch = np.arcsin(up[2])
        yaw = np.arctan2(-direction[1], direction[0])

        # Convert roll, pitch, yaw to a quaternion
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        # Set the pose target
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = entry[0]
        pose_target.position.y = entry[1]
        pose_target.position.z = entry[2]
        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]
        move_group.set_pose_target(pose_target)

        # Planning and executing the motion
        plan_success = move_group.go(wait=True)

        while not plan_success:
            print("Planning failed, trying again")
            move_group.set_pose_target(pose_target)
            plan_success = move_group.go(wait=True)

        move_group.stop()
        move_group.clear_pose_targets()