---
sidebar_position: 2
---

# URDF Basics for Humanoid Robots

## Understanding URDF Structure

URDF (Unified Robot Description Format) is an XML format that describes robot models. The basic structure consists of links connected by joints, forming a kinematic tree. For humanoid robots, this structure typically resembles the human body's skeletal system.

## Links: The Building Blocks

Links represent rigid parts of the robot. Each link contains several sub-elements:

### Visual Properties
Defines how the link appears in visualizers:

```xml
<link name="upper_arm">
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
</link>
```

### Collision Properties
Defines how the link interacts with the physical world:

```xml
<link name="upper_arm">
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>
</link>
```

### Inertial Properties
Defines mass and inertial characteristics for physics simulation:

```xml
<link name="upper_arm">
  <inertial>
    <mass value="0.5"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0001"/>
  </inertial>
</link>
```

## Joints: Connecting the Links

Joints define how links move relative to each other. Common joint types for humanoid robots:

### Fixed Joints
No movement between links:

```xml
<joint name="torso_to_head" type="fixed">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.4" rpy="0 0 0"/>
</joint>
```

### Revolute Joints
Rotational joints with limits (like elbows, knees):

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="1.0" effort="100" velocity="3.0"/>
</joint>
```

### Continuous Joints
Rotational joints without limits (like shoulders, hips):

```xml
<joint name="shoulder_joint" type="continuous">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

## Complete Humanoid Example

Here's a simplified URDF for an upper body humanoid:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_upper_body">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.5"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.5"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="left_shoulder" type="continuous">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.15 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.125"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.00008"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.0" effort="50" velocity="3.0"/>
  </joint>

  <link name="left_hand">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.08 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="left_wrist" type="fixed">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
</robot>
```

## Best Practices for Humanoid URDF

### 1. Naming Conventions
Use consistent naming for symmetric parts:
- `left_upper_arm`, `right_upper_arm`
- `left_leg`, `right_leg`

### 2. Origin and Orientation
- Use consistent coordinate frames (typically X forward, Y left, Z up)
- Keep origins at joint locations when possible

### 3. Inertial Properties
- Calculate realistic mass and inertia values
- Use tools like CAD software to compute inertial properties
- Ensure the center of mass is reasonable for balance

### 4. Joint Limits
- Set appropriate limits based on human anatomy or mechanical constraints
- Include effort and velocity limits for simulation

### 5. Visual vs Collision
- Use simpler geometries for collision than for visual
- Ensure collision models are watertight and manifold

## URDF Validation

Always validate your URDF files:

```bash
# Check for XML syntax errors
xmllint --noout my_robot.urdf

# Check with ROS tools
ros2 run xacro xacro --check-order my_robot.urdf.xacro
```

## Xacro for Complex Humanoid Robots

For complex humanoid robots, use Xacro (XML Macros) to make URDF more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="arm_radius" value="0.05" />

  <!-- Macro for creating arms -->
  <xacro:macro name="arm" params="side parent_link position">
    <link name="${side}_upper_arm">
      <visual>
        <origin xyz="0 0 ${arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${arm_length}" radius="${arm_radius}"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 ${arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${arm_length}" radius="${arm_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <origin xyz="0 0 ${arm_length/2}"/>
        <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0001"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder" type="continuous">
      <parent link="${parent_link}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${position}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:arm side="left" parent_link="torso" position="0.2 0.15 0.3"/>
  <xacro:arm side="right" parent_link="torso" position="0.2 -0.15 0.3"/>
</robot>
```

This approach makes it much easier to create symmetric humanoid robots and maintain consistent properties across similar components.