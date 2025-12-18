---
sidebar_position: 3
---

# rclpy Examples: Agent and Controller Implementation

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing the Python API for developing ROS 2 nodes. For humanoid robotics applications, rclpy enables rapid prototyping and implementation of control algorithms, perception systems, and agent behaviors.

## Basic Node Structure

Every rclpy node follows a similar structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize publishers, subscribers, services, etc.

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

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

## Simple Agent Example: Walking Pattern Generator

This example demonstrates a simple agent that generates walking patterns for a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class WalkingPatternGenerator(Node):
    def __init__(self):
        super().__init__('walking_pattern_generator')

        # Publisher for trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Timer for generating walking pattern
        self.timer = self.create_timer(0.05, self.generate_walking_pattern)  # 20Hz
        self.step_count = 0

    def generate_walking_pattern(self):
        msg = JointTrajectory()
        msg.joint_names = ['left_hip', 'left_knee', 'left_ankle',
                          'right_hip', 'right_knee', 'right_ankle']

        point = JointTrajectoryPoint()

        # Generate simple sinusoidal walking pattern
        phase = self.step_count * 0.1
        left_hip_pos = 0.1 * math.sin(phase)
        right_hip_pos = 0.1 * math.sin(phase + math.pi)

        point.positions = [
            left_hip_pos, 0.0, 0.0,  # Left leg
            right_hip_pos, 0.0, 0.0  # Right leg
        ]

        # Set timing
        point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms
        msg.points = [point]

        self.trajectory_pub.publish(msg)
        self.step_count += 1

        self.get_logger().info(f'Published walking pattern step {self.step_count}')

def main(args=None):
    rclpy.init(args=args)
    node = WalkingPatternGenerator()

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

## Controller Example: Joint Position Controller

This example shows a simple joint position controller that subscribes to trajectory commands:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

class JointPositionController(Node):
    def __init__(self):
        super().__init__('joint_position_controller')

        # Subscriber for trajectory commands
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # Publisher for joint commands (for simulation or hardware interface)
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Subscriber for current joint states
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10
        )

        # Current joint state
        self.current_positions = {}

    def trajectory_callback(self, msg):
        if len(msg.points) > 0:
            target_positions = msg.points[0].positions
            joint_names = msg.joint_names

            # Create command message
            cmd_msg = Float64MultiArray()
            cmd_msg.data = list(target_positions)

            self.command_pub.publish(cmd_msg)
            self.get_logger().info(f'Sent joint commands: {target_positions}')

    def state_callback(self, msg):
        # Update current joint positions
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionController()

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

## Advanced Agent: Behavior Tree Implementation

This example shows a more complex agent using a simple behavior tree pattern:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class BehaviorAgent(Node):
    def __init__(self):
        super().__init__('behavior_agent')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.status_pub = self.create_publisher(String, '/agent_status', 10)

        # Timer for behavior execution
        self.timer = self.create_timer(0.1, self.execute_behavior)

        # Internal state
        self.laser_data = None
        self.current_behavior = "explore"

    def scan_callback(self, msg):
        self.laser_data = msg.ranges

    def execute_behavior(self):
        if self.laser_data is None:
            return

        # Simple behavior tree logic
        if self.is_obstacle_close():
            self.current_behavior = "avoid"
            self.avoid_obstacle()
        else:
            self.current_behavior = "explore"
            self.explore()

        # Publish status
        status_msg = String()
        status_msg.data = self.current_behavior
        self.status_pub.publish(status_msg)

    def is_obstacle_close(self):
        if self.laser_data:
            # Check if any range reading is less than 1 meter
            return any(d < 1.0 and d > 0.1 for d in self.laser_data)
        return False

    def avoid_obstacle(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Turn in place to avoid obstacle
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Avoiding obstacle')

    def explore(self):
        cmd = Twist()
        cmd.linear.x = 0.3  # Move forward
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Exploring')

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorAgent()

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

## Error Handling and Best Practices

### Proper Exception Handling

```python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

        # Declare parameters with defaults
        self.declare_parameter('loop_rate', 10)
        self.declare_parameter('timeout', 5.0)

        # Get parameters safely
        try:
            self.loop_rate = self.get_parameter('loop_rate').value
            self.timeout = self.get_parameter('timeout').value
        except ParameterNotDeclaredException as e:
            self.get_logger().error(f'Parameter not declared: {e}')
            # Use defaults
            self.loop_rate = 10
            self.timeout = 5.0

        self.timer = self.create_timer(1.0/self.loop_rate, self.safe_callback)

    def safe_callback(self):
        try:
            # Your main logic here
            self.get_logger().info('Executing safe callback')
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')
```

## Launch Files for Complex Systems

For humanoid robots with multiple nodes, launch files help coordinate the system:

```python
# launch/humanoid_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='walking_pattern_generator',
            name='walking_generator',
            parameters=[
                {'step_height': 0.05},
                {'step_length': 0.1},
            ]
        ),
        Node(
            package='my_robot_controller',
            executable='joint_position_controller',
            name='joint_controller'
        ),
        Node(
            package='my_robot_controller',
            executable='behavior_agent',
            name='behavior_agent'
        )
    ])
```

## Testing Your Nodes

Use the `rclpy` testing framework to verify your implementations:

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from your_package.your_node import YourNode

class TestYourNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = YourNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_node_initialization(self):
        self.assertIsNotNone(self.node)
        # Add more specific tests
```

These examples provide a foundation for implementing agents and controllers in ROS 2 for humanoid robotics applications. Each example demonstrates key concepts while being directly applicable to humanoid robot systems.