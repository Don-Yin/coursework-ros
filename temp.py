def end_effector_pose(self, coordinates: tuple, orientations: tuple = None):
    group_name = "arm_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_end_effector_link("sphere")

    # Set the planning time to a higher value
    move_group.set_planning_time(10)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = coordinates[0]
    pose_goal.position.y = coordinates[1]
    pose_goal.position.z = coordinates[2]

    if orientations is not None:
        pose_goal.orientation.x = orientations[0]
        pose_goal.orientation.y = orientations[1]
        pose_goal.orientation.z = orientations[2]
        pose_goal.orientation.w = orientations[3]

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)

    while not plan:
        print("Planning failed, trying again")
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()
