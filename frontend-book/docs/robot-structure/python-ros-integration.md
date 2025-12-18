---
sidebar_position: 3
---

# Python-ROS Integration for Robot Structure

## Working with Robot Models in Python

ROS 2 provides several ways to work with robot structure models in Python, particularly for humanoid robots. This section covers how to access, manipulate, and use URDF models within your Python nodes.

## Robot State Publisher

The Robot State Publisher is a crucial ROS 2 node that takes a URDF and joint positions and publishes the appropriate TF transforms. Understanding how to work with it is essential for humanoid robotics.

### Launching Robot State Publisher

```python
# launch_robot_state_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class MockJointStatePublisher(Node):
    def __init__(self):
        super().__init__('mock_joint_state_publisher')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20Hz

        # Initialize joint names (for a simple humanoid)
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        # Initialize joint positions
        self.joint_positions = [0.0] * len(self.joint_names)

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Reading URDF in Python

While ROS 2 doesn't provide direct Python APIs to parse URDF (unlike C++), you can access robot descriptions through ROS parameters:

```python
import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET

class URDFAnalyzer(Node):
    def __init__(self):
        super().__init__('urdf_analyzer')

        # Declare parameter for robot description
        self.declare_parameter('robot_description', '')

        # Get the robot description
        robot_description = self.get_parameter('robot_description').value

        if robot_description:
            self.get_logger().info('Robot description parameter found')
            self.parse_urdf(robot_description)
        else:
            self.get_logger().warn('No robot description parameter found')

    def parse_urdf(self, urdf_string):
        """Parse URDF string and extract information"""
        try:
            root = ET.fromstring(urdf_string)

            # Extract link information
            links = root.findall('link')
            self.get_logger().info(f'Found {len(links)} links')

            for link in links:
                link_name = link.get('name')
                self.get_logger().info(f'  Link: {link_name}')

                # Check for visual, collision, and inertial elements
                if link.find('visual') is not None:
                    self.get_logger().info(f'    Has visual element')
                if link.find('collision') is not None:
                    self.get_logger().info(f'    Has collision element')
                if link.find('inertial') is not None:
                    self.get_logger().info(f'    Has inertial element')

            # Extract joint information
            joints = root.findall('joint')
            self.get_logger().info(f'Found {len(joints)} joints')

            for joint in joints:
                joint_name = joint.get('name')
                joint_type = joint.get('type')
                parent = joint.find('parent').get('link') if joint.find('parent') is not None else 'unknown'
                child = joint.find('child').get('link') if joint.find('child') is not None else 'unknown'

                self.get_logger().info(f'  Joint: {joint_name} ({joint_type}) - {parent} -> {child}')

        except ET.ParseError as e:
            self.get_logger().error(f'Error parsing URDF: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = URDFAnalyzer()

    try:
        rclpy.spin_once(node, timeout_sec=1)  # Just analyze once
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with TF (Transforms)

TF (Transform Library) is how ROS represents coordinate frames and transformations between them. For humanoid robots, this is crucial for understanding the position and orientation of each body part.

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import tf_transformations

class TFExplorer(Node):
    def __init__(self):
        super().__init__('tf_explorer')

        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically check transforms
        self.timer = self.create_timer(1.0, self.check_transforms)

    def check_transforms(self):
        """Check and log available transforms"""
        try:
            # Get all available frames
            frame_names = self.tf_buffer.all_frames_as_string().split('\n')
            self.get_logger().info(f'Available frames: {frame_names}')

            # Try to get a specific transform (e.g., between left foot and base)
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',  # Target frame
                    'left_foot',  # Source frame
                    rclpy.time.Time()  # Get most recent
                )

                # Extract position and orientation
                pos = transform.transform.translation
                quat = transform.transform.rotation

                self.get_logger().info(
                    f'Transform from left_foot to base_link: '
                    f'x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}'
                )

            except Exception as e:
                self.get_logger().info(f'Could not get transform: {e}')

        except Exception as e:
            self.get_logger().error(f'Error in TF exploration: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TFExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Forward and Inverse Kinematics with Python

For humanoid robots, you'll often need to perform kinematic calculations. While ROS 2 doesn't include built-in IK solvers in Python, you can integrate with external libraries:

### Using KDL (Kinematics and Dynamics Library)

```python
# Note: This requires python_orocos_kdl package
import rclpy
from rclpy.node import Node
import PyKDL as kdl
from geometry_msgs.msg import Pose, Point, Quaternion

class KinematicsSolver(Node):
    def __init__(self):
        super().__init__('kinematics_solver')

        # Example: Create a simple chain for a humanoid arm
        # This is a simplified example - real implementation would be more complex
        self.chain = kdl.Chain()

        # Add segments to the chain (simplified)
        # In practice, you'd build this from your URDF
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotZ),  # Joint type
            kdl.Frame(kdl.Vector(0, 0, 0.1))  # Link transform
        ))
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotY),
            kdl.Frame(kdl.Vector(0, 0, 0.3))  # Upper arm length
        ))
        self.chain.addSegment(kdl.Segment(
            kdl.Joint(kdl.Joint.RotY),
            kdl.Frame(kdl.Vector(0, 0, 0.25))  # Lower arm length
        ))

        # Create solvers
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)

    def forward_kinematics(self, joint_angles):
        """Calculate end-effector position from joint angles"""
        # Convert joint angles to KDL joint array
        joint_array = kdl.JntArray(len(joint_angles))
        for i, angle in enumerate(joint_angles):
            joint_array[i] = angle

        # Calculate forward kinematics
        end_effector_pose = kdl.Frame()
        result = self.fk_solver.JntToCart(joint_array, end_effector_pose)

        if result >= 0:
            # Extract position and orientation
            pos = end_effector_pose.p
            rot = end_effector_pose.M
            return (pos[0], pos[1], pos[2]), (rot.GetQuaternion())
        else:
            return None, None

    def inverse_kinematics(self, target_pose):
        """Calculate joint angles for desired end-effector pose"""
        # This is a simplified example
        # Real IK would be more complex and require more sophisticated solvers
        pass

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsSolver()

    # Example usage
    joint_angles = [0.1, 0.5, -0.3]  # Example joint angles
    pos, rot = node.forward_kinematics(joint_angles)

    if pos:
        node.get_logger().info(f'End-effector position: {pos}')
    else:
        node.get_logger().error('Forward kinematics failed')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Using MoveIt2 with Python

MoveIt2 is the standard motion planning framework for ROS 2. While primarily C++, it has Python interfaces:

```python
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import Constraints, JointConstraint

class MoveItController(Node):
    def __init__(self):
        super().__init__('moveit_controller')

        # Create action client for MoveGroup
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            'move_group'
        )

        # Wait for the action server to be available
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()

    def move_to_pose(self, group_name, target_pose):
        """Send a pose goal to MoveIt2"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name

        # Set target pose constraint
        pose_constraint = Constraints()
        # Add pose constraint here

        goal_msg.request.goal_constraints.append(pose_constraint)

        # Send goal
        future = self.move_group_client.send_goal_async(goal_msg)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = MoveItController()

    # Example target pose
    target_pose = Pose()
    target_pose.position = Point(x=0.5, y=0.0, z=1.0)
    target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # Move to pose (this is a simplified example)
    # In practice, you'd need to set up MoveIt2 properly first

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## URDF Processing Pipeline

Here's a complete example that demonstrates processing URDF in a Python node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import xml.etree.ElementTree as ET

class URDFProcessor(Node):
    def __init__(self):
        super().__init__('urdf_processor')

        # Declare and get robot description
        self.declare_parameter('robot_description', '')
        robot_description = self.get_parameter('robot_description').value

        if robot_description:
            self.robot_model = self.parse_urdf(robot_description)
            self.get_logger().info('URDF loaded successfully')
        else:
            self.robot_model = None
            self.get_logger().warn('No URDF provided')

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.command_sub = self.create_subscription(
            Twist, 'cmd_vel', self.command_callback, 10
        )

        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_robot_state)

    def parse_urdf(self, urdf_string):
        """Parse URDF and create a simple model representation"""
        try:
            root = ET.fromstring(urdf_string)

            model = {
                'name': root.get('name'),
                'links': {},
                'joints': {}
            }

            # Parse links
            for link_elem in root.findall('link'):
                link_name = link_elem.get('name')
                link_data = {
                    'visual': link_elem.find('visual') is not None,
                    'collision': link_elem.find('collision') is not None,
                    'inertial': link_elem.find('inertial') is not None
                }
                model['links'][link_name] = link_data

            # Parse joints
            for joint_elem in root.findall('joint'):
                joint_name = joint_elem.get('name')
                joint_type = joint_elem.get('type')
                parent_link = joint_elem.find('parent').get('link') if joint_elem.find('parent') is not None else None
                child_link = joint_elem.find('child').get('link') if joint_elem.find('child') is not None else None

                joint_data = {
                    'type': joint_type,
                    'parent': parent_link,
                    'child': child_link
                }
                model['joints'][joint_name] = joint_data

            return model

        except ET.ParseError as e:
            self.get_logger().error(f'Error parsing URDF: {e}')
            return None

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        if self.robot_model:
            # Validate that joint names in message match URDF
            for joint_name in msg.name:
                if joint_name not in self.robot_model['joints']:
                    self.get_logger().warn(f'Joint {joint_name} not found in URDF')

    def command_callback(self, msg):
        """Process command messages"""
        self.get_logger().info(f'Received command: linear={msg.linear.x}, angular={msg.angular.z}')

    def process_robot_state(self):
        """Process robot state based on URDF"""
        if self.robot_model:
            self.get_logger().info(f'Processing robot model: {self.robot_model["name"]}')

def main(args=None):
    rclpy.init(args=args)
    node = URDFProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File for URDF Integration

```python
# launch/urdf_integration.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    urdf_path = LaunchConfiguration('urdf_path')
    declare_urdf_path = DeclareLaunchArgument(
        'urdf_path',
        default_value=os.path.join(
            get_package_share_directory('my_humanoid_package'),
            'urdf',
            'humanoid.urdf'
        ),
        description='Path to the URDF file'
    )

    # Read URDF file
    with open(LaunchConfiguration('urdf_path').perform({}), 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        declare_urdf_path,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher (for GUI control)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Your custom URDF processing node
        Node(
            package='my_humanoid_package',
            executable='urdf_processor',
            name='urdf_processor',
            parameters=[{'robot_description': robot_description}]
        )
    ])
```

This integration allows you to work with robot structure models in Python, enabling sophisticated control and analysis of humanoid robot systems based on their URDF descriptions.