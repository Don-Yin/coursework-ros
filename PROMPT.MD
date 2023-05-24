
# 3D volumes

# Moveit setup

# Moveit package created directory
rosbox@rosbox-VirtualBox:~/coursework-ros$ tree src/move_robot_moveit_config -L 3
src/move_robot_moveit_config
├── CMakeLists.txt
├── config
│   ├── cartesian_limits.yaml
│   ├── chomp_planning.yaml
│   ├── fake_controllers.yaml
│   ├── gazebo_controllers.yaml
│   ├── gazebo_improved_robot.urdf
│   ├── improved_robot.srdf
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── ompl_planning.yaml
│   ├── ros_controllers.yaml
│   ├── sensors_3d.yaml
│   ├── simple_moveit_controllers.yaml
│   └── stomp_planning.yaml
├── launch
│   ├── chomp_planning_pipeline.launch.xml
│   ├── default_warehouse_db.launch
│   ├── demo_gazebo.launch
│   ├── demo.launch
│   ├── fake_moveit_controller_manager.launch.xml
│   ├── gazebo.launch
│   ├── improved_robot_moveit_sensor_manager.launch.xml
│   ├── joystick_control.launch
│   ├── move_group.launch
│   ├── moveit.rviz
│   ├── moveit_rviz.launch
│   ├── ompl-chomp_planning_pipeline.launch.xml
│   ├── ompl_planning_pipeline.launch.xml
│   ├── pilz_industrial_motion_planner_planning_pipeline.launch.xml
│   ├── planning_context.launch
│   ├── planning_pipeline.launch.xml
│   ├── ros_controllers.launch
│   ├── ros_control_moveit_controller_manager.launch.xml
│   ├── run_benchmark_ompl.launch
│   ├── sensor_manager.launch.xml
│   ├── setup_assistant.launch
│   ├── simple_moveit_controller_manager.launch.xml
│   ├── stomp_planning_pipeline.launch.xml
│   ├── trajectory_execution.launch.xml
│   ├── warehouse.launch
│   └── warehouse_settings.launch.xml
└── package.xml