
# System
Ros Noetic

# 3D volumes
data/BrainParcellation/
    cortex.nii.gz
    r_hippo.nii.gz
    ventricles.nii.gz
    vessels.nii.gz

# Moveit setup
```roslaunch moveit_setup_assistant setup_assistant.launch```
```catkin_make```
```source devel/setup.sh```

Note: the assets/arm.urdf is used for the moveit_setup_assistant, the content of the URDF is shown below:

<?xml version="1.0"?>
<robot name="improved_robot">

  <disable_collisions link1="arm1" link2="arm1b" reason="Adjacent" />
  <disable_collisions link1="arm1b" link2="arm2" reason="Adjacent" />

  <link name="base_link">
    <visual>
      <geometry>
        <box size="4 4 1"/>
      </geometry>
    </visual>
  </link>

  <link name="rotator1">
    <visual>
      <geometry>
        <cylinder radius="2" length="1"/>
      </geometry>
    </visual>
  </link>

  <link name="pivot1">
    <visual>
      <origin xyz="0 0 0" rpy="0 1.57 0"/>
      <geometry>
        <cylinder radius="1" length="2"/>
      </geometry>
    </visual>
  </link>

  <link name ="arm1">
    <visual>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
      <origin xyz="0 0 3.75" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="1" length="6"/>
      </geometry>
    </visual>
  </link>

  <link name="pivot1b">
    <visual>
      <origin xyz="0 0 0" rpy="0 1.57 0"/>
      <geometry>
        <cylinder radius="1" length="2"/>
      </geometry>
    </visual>
  </link>

  <link name ="arm1b">
    <visual>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
      <origin xyz="0 0 3.75" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="1" length="6"/>
      </geometry>
    </visual>
  </link>

  <link name="pivot2">
    <visual>
      <origin xyz="0 0 0" rpy="0 1.57 0"/>
      <geometry>
        <cylinder radius="1" length="2"/>
      </geometry>
    </visual>
  </link>

  <link name ="arm2">
    <visual>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
      <origin xyz="0 0 4" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="1" length="6"/>
      </geometry>
    </visual>
  </link>

  <link name ="end_effector">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="1"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_link_to_rotator1" type="continuous">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="rotator1"/>
  </joint>

  <joint name="rotator1_to_pivot1" type="fixed">
    <origin xyz="0 0 1.5" rpy="0 0 0"/>
    <parent link="rotator1"/>
    <child link="pivot1"/>
  </joint>

  <joint name="pivot1_to_arm1" type="revolute">
    <parent link="pivot1"/>
    <child link="arm1"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>

  <joint name="arm1_to_pivot1b" type="continuous">
    <parent link="arm1"/>
    <child link="pivot1b"/>
    <origin xyz="0 0 7.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="pivot1b_to_arm1b" type="revolute">
    <parent link="pivot1b"/>
    <child link="arm1b"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>
  <joint name="arm1b_to_pivot2" type="continuous">
    <parent link="arm1b"/>
    <child link="pivot2"/>
    <origin xyz="0 0 7.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="pivot2_to_arm2" type="revolute">
    <parent link="pivot2"/>
    <child link="arm2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>
  <joint name="arm2_to_end_effector" type="fixed">
    <origin xyz="0 0 7.5" rpy="0 0 0"/>
    <parent link="arm2"/>
    <child link="end_effector"/>
  </joint>
</robot>



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

# Origin and target points
origin: (208.596, 133.917, 115.449)
target: (158.0, 128.0, 114.0)

# What I want to achieve
- visualize the brain volumes in Rviz (converting to point cloud or mesh; mesh - .stl or .dae)
- visualize the robot in the same Rviz window with the brain volumes
- place the end_effector on the origin point
- move the end_effector along a straight path to the target point

# Your task
Answer
- what is the easiest way of doing what I want to achieve?
- can this be achieved in a single launch file?

