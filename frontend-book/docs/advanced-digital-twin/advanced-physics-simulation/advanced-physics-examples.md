---
sidebar_position: 10
---

# Advanced Physics Examples

This page provides practical Gazebo configurations demonstrating advanced physics simulation techniques for humanoid robots.

## Example 1: Humanoid Robot with Advanced Physics Properties

Here's a complete SDF model of a simple humanoid robot with advanced physics properties:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="advanced_humanoid">
    <!-- Robot base/torso -->
    <link name="base_link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.3</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.4</iyy>
          <iyz>0.0</iyz>
          <izz>0.2</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </collision>

      <!-- Advanced surface properties -->
      <surface>
        <contact>
          <ode>
            <soft_cfm>0.0001</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <kp>100000000.0</kp>
            <kd>1.0</kd>
            <max_vel>100.0</max_vel>
            <min_depth>0.001</min_depth>
          </ode>
        </contact>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
            <fdir1>0 0 1</fdir1>
          </ode>
        </friction>
      </surface>
    </link>

    <!-- Head link -->
    <link name="head">
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </visual>

      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
    </link>

    <!-- Neck joint -->
    <joint name="neck_joint" type="revolute">
      <parent>base_link</parent>
      <child>head</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>10.0</effort>
          <velocity>1.0</velocity>
        </limit>
        <dynamics>
          <damping>0.5</damping>
          <friction>0.1</friction>
        </dynamics>
      </axis>
      <pose>0.0 0.0 0.3 0 0 0</pose>
    </joint>

    <!-- Left upper arm -->
    <link name="left_upper_arm">
      <inertial>
        <mass>1.5</mass>
        <inertia>
          <ixx>0.02</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.02</iyy>
          <iyz>0.0</iyz>
          <izz>0.005</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <capsule>
            <radius>0.04</radius>
            <length>0.25</length>
          </capsule>
        </geometry>
      </visual>

      <collision name="collision">
        <geometry>
          <capsule>
            <radius>0.04</radius>
            <length>0.25</length>
          </capsule>
        </geometry>
      </collision>

      <surface>
        <contact>
          <ode>
            <soft_cfm>0.0001</soft_cfm>
            <soft_erp>0.2</soft_erp>
          </ode>
        </contact>
        <friction>
          <ode>
            <mu>0.6</mu>
            <mu2>0.6</mu2>
          </ode>
        </friction>
      </surface>
    </link>

    <!-- Left shoulder joint -->
    <joint name="left_shoulder_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_upper_arm</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>20.0</effort>
          <velocity>2.0</velocity>
        </limit>
        <dynamics>
          <damping>0.8</damping>
        </dynamics>
      </axis>
      <pose>-0.15 0.1 0.1 0 0 0</pose>
    </joint>
  </model>
</sdf>
```

## Example 2: Advanced World Configuration

A world file with advanced physics parameters:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="advanced_physics_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>

      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.000001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Include the advanced humanoid model -->
    <include>
      <uri>model://advanced_humanoid</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

    <!-- Ground plane with advanced friction -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Example 3: Python Controller for Physics Testing

A Python script to test and validate physics behavior:

```python
#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class AdvancedPhysicsTester:
    def __init__(self):
        rospy.init_node('advanced_physics_tester')

        # Publishers for joint control
        self.joint_publishers = {}
        joint_names = ['left_shoulder_joint', 'neck_joint']

        for joint_name in joint_names:
            pub = rospy.Publisher(f'/joint_effort_controller/{joint_name}/command',
                                Float64, queue_size=10)
            self.joint_publishers[joint_name] = pub

        # Subscriber for joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # Service proxy for model state
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.rate = rospy.Rate(100)  # 100 Hz

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        # Store current joint states for analysis
        self.current_joint_states = msg

    def apply_advanced_physics_test(self):
        """Apply various physics tests to validate behavior"""
        # Test 1: Apply oscillating torque to shoulder joint
        t = rospy.Time.now().to_sec()
        torque = 5.0 * math.sin(2 * math.pi * 0.5 * t)  # 0.5 Hz oscillation

        cmd_msg = Float64()
        cmd_msg.data = torque
        self.joint_publishers['left_shoulder_joint'].publish(cmd_msg)

        # Test 2: Apply periodic impulse to neck joint
        impulse_phase = (t * 2) % 4  # Every 2 seconds
        if 0 < impulse_phase < 0.1:  # Short impulse
            neck_torque = 2.0
            cmd_msg.data = neck_torque
            self.joint_publishers['neck_joint'].publish(cmd_msg)

    def validate_physics_behavior(self):
        """Validate that physics behavior is realistic"""
        if hasattr(self, 'current_joint_states'):
            # Check for realistic joint velocities (not too high)
            for i, name in enumerate(self.current_joint_states.name):
                if name == 'left_shoulder_joint':
                    velocity = self.current_joint_states.velocity[i]
                    if abs(velocity) > 10.0:  # Unrealistic velocity
                        rospy.logwarn(f"Unrealistic velocity for {name}: {velocity}")

    def run(self):
        """Main execution loop"""
        rospy.loginfo("Starting advanced physics testing...")

        while not rospy.is_shutdown():
            self.apply_advanced_physics_test()
            self.validate_physics_behavior()
            self.rate.sleep()

if __name__ == '__main__':
    tester = AdvancedPhysicsTester()
    tester.run()
```

## Example 4: Unity Synchronization Script

A Unity C# script to demonstrate integration with Gazebo physics:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class AdvancedPhysicsSynchronizer : MonoBehaviour
{
    [System.Serializable]
    public class LinkMapping
    {
        public string gazeboName;
        public GameObject unityObject;
    }

    public List<LinkMapping> linkMappings = new List<LinkMapping>();
    public float interpolationSpeed = 10.0f;
    public bool useInterpolation = true;

    private Dictionary<string, PoseData> targetPoses = new Dictionary<string, PoseData>();
    private Dictionary<string, PoseData> currentPoses = new Dictionary<string, PoseData>();

    [System.Serializable]
    public class PoseData
    {
        public Vector3 position;
        public Quaternion rotation;
        public float timestamp;

        public PoseData(Vector3 pos, Quaternion rot, float time)
        {
            position = pos;
            rotation = rot;
            timestamp = time;
        }
    }

    void Start()
    {
        // Initialize current poses to match initial Unity positions
        foreach (var mapping in linkMappings)
        {
            if (mapping.unityObject != null)
            {
                currentPoses[mapping.gazeboName] = new PoseData(
                    mapping.unityObject.transform.position,
                    mapping.unityObject.transform.rotation,
                    Time.time
                );
            }
        }
    }

    public void UpdateTargetPose(string gazeboName, Vector3 position, Quaternion rotation)
    {
        targetPoses[gazeboName] = new PoseData(position, rotation, Time.time);
    }

    void Update()
    {
        // Update all mapped objects
        foreach (var mapping in linkMappings)
        {
            if (mapping.unityObject != null && targetPoses.ContainsKey(mapping.gazeboName))
            {
                PoseData targetPose = targetPoses[mapping.gazeboName];

                if (useInterpolation)
                {
                    // Smooth interpolation to target pose
                    mapping.unityObject.transform.position = Vector3.Lerp(
                        mapping.unityObject.transform.position,
                        targetPose.position,
                        Time.deltaTime * interpolationSpeed
                    );

                    mapping.unityObject.transform.rotation = Quaternion.Slerp(
                        mapping.unityObject.transform.rotation,
                        targetPose.rotation,
                        Time.deltaTime * interpolationSpeed
                    );
                }
                else
                {
                    // Direct setting (no interpolation)
                    mapping.unityObject.transform.position = targetPose.position;
                    mapping.unityObject.transform.rotation = targetPose.rotation;
                }
            }
        }
    }

    // Helper method to convert from Gazebo to Unity coordinate system
    public static Vector3 ConvertGazeboToUnityPosition(Vector3 gazeboPos)
    {
        return new Vector3(gazeboPos.y, gazeboPos.z, gazeboPos.x);
    }

    public static Quaternion ConvertGazeboToUnityRotation(Quaternion gazeboRot)
    {
        return new Quaternion(
            gazeboRot.y,
            gazeboRot.z,
            gazeboRot.x,
            -gazeboRot.w
        );
    }
}
```

## Performance Optimization Tips

### For Gazebo:
1. **Reduce Physics Update Rate**: Balance between accuracy and performance
2. **Limit Solver Iterations**: Start with lower values and increase as needed
3. **Use Proper Inertial Values**: Avoid extremely high or low mass/inertia values
4. **Optimize Collision Geometry**: Use simpler shapes where possible

### For Unity:
1. **Use Object Pooling**: For frequently instantiated objects
2. **Optimize Rendering**: Use Level of Detail (LOD) for distant objects
3. **Batch Draw Calls**: Combine similar objects for better rendering performance
4. **Physics Optimization**: Adjust fixed timestep in Time settings

These examples demonstrate practical implementations of advanced physics concepts in both Gazebo and Unity, showing how to create realistic humanoid robot simulations with proper physics properties and integration between platforms.