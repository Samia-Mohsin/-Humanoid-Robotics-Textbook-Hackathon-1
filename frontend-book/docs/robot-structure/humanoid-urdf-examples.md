---
sidebar_position: 4
---

# URDF Examples for Humanoid Robots with Simulation Readiness

## Complete Humanoid URDF Example

Here's a comprehensive URDF for a basic humanoid robot that's ready for simulation:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Mass properties -->
  <xacro:property name="base_mass" value="10.0" />
  <xacro:property name="torso_mass" value="5.0" />
  <xacro:property name="head_mass" value="1.0" />
  <xacro:property name="arm_mass" value="0.5" />
  <xacro:property name="leg_mass" value="1.0" />
  <xacro:property name="foot_mass" value="0.2" />

  <!-- Dimensions -->
  <xacro:property name="torso_height" value="0.6" />
  <xacro:property name="torso_width" value="0.3" />
  <xacro:property name="torso_depth" value="0.2" />
  <xacro:property name="head_radius" value="0.1" />
  <xacro:property name="upper_arm_length" value="0.3" />
  <xacro:property name="lower_arm_length" value="0.25" />
  <xacro:property name="upper_arm_radius" value="0.05" />
  <xacro:property name="lower_arm_radius" value="0.04" />
  <xacro:property name="upper_leg_length" value="0.4" />
  <xacro:property name="lower_leg_length" value="0.4" />
  <xacro:property name="upper_leg_radius" value="0.07" />
  <xacro:property name="lower_leg_radius" value="0.06" />
  <xacro:property name="foot_size" value="0.15 0.08 0.05" />

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="dark_grey">
    <color rgba="0.3 0.3 0.3 1.0"/>
  </material>
  <material name="light_grey">
    <color rgba="0.8 0.8 0.8 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 ${torso_height/2}"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${torso_height/2}"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${torso_mass}"/>
      <origin xyz="0 0 ${torso_height/2}"/>
      <inertia ixx="${torso_mass/12.0 * (torso_depth*torso_depth + torso_height*torso_height)}"
               ixy="0" ixz="0"
               iyy="${torso_mass/12.0 * (torso_width*torso_width + torso_height*torso_height)}"
               iyz="0"
               izz="${torso_mass/12.0 * (torso_width*torso_width + torso_depth*torso_depth)}"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 ${head_radius}"/>
      <geometry>
        <sphere radius="${head_radius}"/>
      </geometry>
      <material name="light_grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${head_radius}"/>
      <geometry>
        <sphere radius="${head_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${head_mass}"/>
      <origin xyz="0 0 ${head_radius}"/>
      <inertia ixx="${0.4 * head_mass * head_radius * head_radius}"
               ixy="0" ixz="0"
               iyy="${0.4 * head_mass * head_radius * head_radius}"
               iyz="0"
               izz="${0.4 * head_mass * head_radius * head_radius}"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_height}"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="10" velocity="${M_PI/2}"/>
  </joint>

  <!-- Left Arm -->
  <xacro:macro name="arm" params="side parent_link position">
    <!-- Upper Arm -->
    <link name="${side}_upper_arm">
      <visual>
        <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${upper_arm_length}" radius="${upper_arm_radius}"/>
        </geometry>
        <material name="blue"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${upper_arm_length}" radius="${upper_arm_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${arm_mass}"/>
        <origin xyz="0 0 ${upper_arm_length/2}"/>
        <inertia ixx="${arm_mass/12.0 * (3*upper_arm_radius*upper_arm_radius + upper_arm_length*upper_arm_length)}"
                 ixy="0" ixz="0"
                 iyy="${arm_mass/12.0 * (3*upper_arm_radius*upper_arm_radius + upper_arm_length*upper_arm_length)}"
                 iyz="0"
                 izz="${arm_mass * upper_arm_radius * upper_arm_radius / 2.0}"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="continuous">
      <parent link="${parent_link}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${position}"/>
      <axis xyz="0 1 0"/>
    </joint>

    <!-- Lower Arm -->
    <link name="${side}_lower_arm">
      <visual>
        <origin xyz="0 0 ${lower_arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${lower_arm_length}" radius="${lower_arm_radius}"/>
        </geometry>
        <material name="blue"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${lower_arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${lower_arm_length}" radius="${lower_arm_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${arm_mass*0.8}"/>
        <origin xyz="0 0 ${lower_arm_length/2}"/>
        <inertia ixx="${(arm_mass*0.8)/12.0 * (3*lower_arm_radius*lower_arm_radius + lower_arm_length*lower_arm_length)}"
                 ixy="0" ixz="0"
                 iyy="${(arm_mass*0.8)/12.0 * (3*lower_arm_radius*lower_arm_radius + lower_arm_length*lower_arm_length)}"
                 iyz="0"
                 izz="${(arm_mass*0.8) * lower_arm_radius * lower_arm_radius / 2.0}"/>
      </inertial>
    </link>

    <joint name="${side}_elbow_joint" type="revolute">
      <parent link="${side}_upper_arm"/>
      <child link="${side}_lower_arm"/>
      <origin xyz="0 0 ${upper_arm_length}"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10" velocity="${M_PI/2}"/>
    </joint>

    <!-- Hand -->
    <link name="${side}_hand">
      <visual>
        <origin xyz="0 0 0.025"/>
        <geometry>
          <box size="0.08 0.08 0.05"/>
        </geometry>
        <material name="light_grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 0.025"/>
        <geometry>
          <box size="0.08 0.08 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <origin xyz="0 0 0.025"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>

    <joint name="${side}_wrist_joint" type="revolute">
      <parent link="${side}_lower_arm"/>
      <child link="${side}_hand"/>
      <origin xyz="0 0 ${lower_arm_length}"/>
      <axis xyz="0 0 1"/>
      <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="5" velocity="${M_PI/2}"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate arms -->
  <xacro:arm side="left" parent_link="torso" position="0.2 0.15 ${torso_height*0.7}"/>
  <xacro:arm side="right" parent_link="torso" position="0.2 -0.15 ${torso_height*0.7}"/>

  <!-- Left Leg -->
  <xacro:macro name="leg" params="side parent_link position">
    <!-- Upper Leg (Thigh) -->
    <link name="${side}_upper_leg">
      <visual>
        <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${upper_leg_length}" radius="${upper_leg_radius}"/>
        </geometry>
        <material name="red"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${upper_leg_length}" radius="${upper_leg_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${leg_mass}"/>
        <origin xyz="0 0 ${-upper_leg_length/2}"/>
        <inertia ixx="${leg_mass/12.0 * (3*upper_leg_radius*upper_leg_radius + upper_leg_length*upper_leg_length)}"
                 ixy="0" ixz="0"
                 iyy="${leg_mass/12.0 * (3*upper_leg_radius*upper_leg_radius + upper_leg_length*upper_leg_length)}"
                 iyz="0"
                 izz="${leg_mass * upper_leg_radius * upper_leg_radius / 2.0}"/>
      </inertial>
    </link>

    <joint name="${side}_hip_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_upper_leg"/>
      <origin xyz="${position}"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50" velocity="${M_PI/2}"/>
    </joint>

    <!-- Lower Leg -->
    <link name="${side}_lower_leg">
      <visual>
        <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${lower_leg_length}" radius="${lower_leg_radius}"/>
        </geometry>
        <material name="red"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${lower_leg_length}" radius="${lower_leg_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${leg_mass*0.8}"/>
        <origin xyz="0 0 ${-lower_leg_length/2}"/>
        <inertia ixx="${(leg_mass*0.8)/12.0 * (3*lower_leg_radius*lower_leg_radius + lower_leg_length*lower_leg_length)}"
                 ixy="0" ixz="0"
                 iyy="${(leg_mass*0.8)/12.0 * (3*lower_leg_radius*lower_leg_radius + lower_leg_length*lower_leg_length)}"
                 iyz="0"
                 izz="${(leg_mass*0.8) * lower_leg_radius * lower_leg_radius / 2.0}"/>
      </inertial>
    </link>

    <joint name="${side}_knee_joint" type="revolute">
      <parent link="${side}_upper_leg"/>
      <child link="${side}_lower_leg"/>
      <origin xyz="0 0 ${-upper_leg_length}"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/2}" upper="0" effort="50" velocity="${M_PI/2}"/>
    </joint>

    <!-- Foot -->
    <link name="${side}_foot">
      <visual>
        <origin xyz="0 0 0"/>
        <geometry>
          <box size="${foot_size}"/>
        </geometry>
        <material name="light_grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 0"/>
        <geometry>
          <box size="${foot_size}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${foot_mass}"/>
        <origin xyz="0 0 0"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${side}_ankle_joint" type="revolute">
      <parent link="${side}_lower_leg"/>
      <child link="${side}_foot"/>
      <origin xyz="0 0 ${-lower_leg_length}"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="10" velocity="${M_PI/2}"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate legs -->
  <xacro:leg side="left" parent_link="torso" position="0 -0.08 0"/>
  <xacro:leg side="right" parent_link="torso" position="0 0.08 0"/>

  <!-- Gazebo plugin for simulation -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_humanoid</robotNamespace>
    </plugin>
  </gazebo>

  <!-- Gazebo materials -->
  <gazebo reference="torso">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/LightGrey</material>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="right_upper_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_lower_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="right_lower_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_upper_leg">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_upper_leg">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_lower_leg">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_lower_leg">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_foot">
    <material>Gazebo/LightGrey</material>
  </gazebo>

  <gazebo reference="right_foot">
    <material>Gazebo/LightGrey</material>
  </gazebo>

</robot>
```

## Simulation Configuration Files

### Gazebo Controller Configuration

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_leg_controller:
      type: effort_controllers/JointGroupEffortController

    right_leg_controller:
      type: effort_controllers/JointGroupEffortController

    left_arm_controller:
      type: effort_controllers/JointGroupEffortController

    right_arm_controller:
      type: effort_controllers/JointGroupEffortController

left_leg_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint

right_leg_controller:
  ros__parameters:
    joints:
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint

left_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_elbow_joint
      - left_wrist_joint

right_arm_controller:
  ros__parameters:
    joints:
      - right_shoulder_joint
      - right_elbow_joint
      - right_wrist_joint
```

### Robot Semantic Description (for MoveIt2)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!-- robot.srdf -->
<robot name="simple_humanoid">
  <!-- Groups -->
  <group name="left_arm">
    <chain base_link="torso" tip_link="left_hand"/>
  </group>

  <group name="right_arm">
    <chain base_link="torso" tip_link="right_hand"/>
  </group>

  <group name="left_leg">
    <chain base_link="torso" tip_link="left_foot"/>
  </group>

  <group name="right_leg">
    <chain base_link="torso" tip_link="right_foot"/>
  </group>

  <group name="both_arms">
    <group name="left_arm"/>
    <group name="right_arm"/>
  </group>

  <group name="whole_body">
    <group name="both_arms"/>
    <group name="left_leg"/>
    <group name="right_leg"/>
  </group>

  <!-- Virtual joints for base -->
  <virtual_joint name="virtual_joint" type="planar" parent_frame="odom" child_link="base_link"/>

  <!-- Disable collision -->
  <disable_collisions link1="torso" link2="left_upper_arm"/>
  <disable_collisions link1="torso" link2="right_upper_arm"/>
  <!-- Add more collision pairs as needed -->

</robot>
```

## Launch Files for Simulation

### Gazebo Simulation Launch

```python
# launch/humanoid_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    urdf_model_path = LaunchConfiguration('urdf_model')
    declare_urdf_model_path = DeclareLaunchArgument(
        'urdf_model',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_humanoid_package'),
            'urdf',
            'simple_humanoid.urdf.xacro'
        ]),
        description='URDF path'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
    )

    # Robot State Publisher
    with open(urdf_model_path.perform({}), 'r') as infp:
        robot_desc = infp.read()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_humanoid',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_urdf_model_path,
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])
```

### Controller Launch

```python
# launch/humanoid_controllers.launch.py
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_humanoid_package'),
                'config',
                'controllers.yaml'
            ])
        ],
        output='both'
    )

    # Load controllers after controller manager starts
    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        )
    )

    load_left_leg_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['left_leg_controller'],
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        )
    )

    load_right_leg_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['right_leg_controller'],
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        )
    )

    load_left_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['left_arm_controller'],
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        )
    )

    load_right_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['right_arm_controller'],
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        )
    )

    return LaunchDescription([
        controller_manager,
        load_joint_state_broadcaster,
        load_left_leg_controller,
        load_right_leg_controller,
        load_left_arm_controller,
        load_right_arm_controller
    ])
```

## Testing URDF in Simulation

To test your URDF:

1. **Validate the URDF:**
```bash
# Check for XML syntax
xmllint --noout simple_humanoid.urdf.xacro

# Check with xacro
ros2 run xacro xacro simple_humanoid.urdf.xacro
```

2. **Visualize the robot:**
```bash
# Use rviz to visualize
ros2 run rviz2 rviz2

# Use check_urdf to validate kinematics
check_urdf simple_humanoid.urdf
```

3. **Run the simulation:**
```bash
# Launch Gazebo simulation
ros2 launch my_humanoid_package humanoid_gazebo.launch.py

# Launch controllers
ros2 launch my_humanoid_package humanoid_controllers.launch.py
```

This comprehensive example provides a complete humanoid robot URDF that's ready for simulation with proper inertial properties, joint limits, and Gazebo integration.