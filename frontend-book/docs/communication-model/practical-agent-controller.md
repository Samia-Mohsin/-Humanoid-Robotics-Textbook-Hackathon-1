---
sidebar_position: 4
---

# Practical rclpy-Based Agent and Controller Examples

## Complete Humanoid Control System Example

This section presents a complete example of a humanoid robot control system that demonstrates the integration of multiple nodes working together to achieve complex behaviors.

### Complete Humanoid Controller Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray, Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Timers
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz for control
        self.status_timer = self.create_timer(1.0, self.publish_status)   # 1Hz for status

        # Internal state
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.imu_data = None
        self.balance_active = False
        self.target_velocity = Twist()
        self.control_mode = 'idle'  # idle, walk, balance, manual

        self.get_logger().info('Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.imu_data = msg

    def control_loop(self):
        """Main control loop running at 100Hz"""
        if self.control_mode == 'balance':
            self.execute_balance_control()
        elif self.control_mode == 'walk':
            self.execute_walking_pattern()
        elif self.control_mode == 'manual':
            self.execute_manual_control()

    def execute_balance_control(self):
        """Implement balance control algorithm"""
        if self.imu_data is None:
            return

        # Simple PD controller for balance
        roll = math.atan2(2.0 * (self.imu_data.orientation.w * self.imu_data.orientation.x +
                                self.imu_data.orientation.y * self.imu_data.orientation.z),
                         1.0 - 2.0 * (self.imu_data.orientation.x**2 + self.imu_data.orientation.y**2))

        pitch = math.asin(2.0 * (self.imu_data.orientation.w * self.imu_data.orientation.y -
                                self.imu_data.orientation.z * self.imu_data.orientation.x))

        # Calculate correction torques based on IMU data
        kp = 5.0  # Proportional gain
        kd = 1.0  # Derivative gain

        roll_correction = -kp * roll
        pitch_correction = -kp * pitch

        # Publish joint commands based on corrections
        cmd_msg = Float64MultiArray()
        # This is a simplified example - real implementation would be more complex
        cmd_msg.data = [roll_correction, pitch_correction, 0.0, 0.0, 0.0, 0.0]  # Example for 6 joints
        self.joint_cmd_pub.publish(cmd_msg)

    def execute_walking_pattern(self):
        """Generate walking pattern based on target velocity"""
        if self.target_velocity.linear.x == 0 and self.target_velocity.angular.z == 0:
            # Stop walking if no target velocity
            self.control_mode = 'balance'
            return

        # Generate walking trajectory based on target velocity
        msg = JointTrajectory()
        msg.joint_names = ['left_hip', 'left_knee', 'left_ankle',
                          'right_hip', 'right_knee', 'right_ankle']

        point = JointTrajectoryPoint()

        # Simplified walking pattern - in practice this would be much more complex
        # Calculate based on target velocity
        step_size = self.target_velocity.linear.x * 0.1  # Scale factor
        turn_factor = self.target_velocity.angular.z * 0.05  # Scale factor

        # Generate appropriate joint positions for walking
        point.positions = [
            step_size, 0.0, 0.0,  # Left leg
            step_size, 0.0, 0.0   # Right leg
        ]

        point.time_from_start = Duration(sec=0, nanosec=10000000)  # 10ms
        msg.points = [point]

        self.trajectory_pub.publish(msg)

    def execute_manual_control(self):
        """Execute manual control commands"""
        # Implementation for manual joint control
        pass

    def publish_status(self):
        """Publish robot status at 1Hz"""
        status_msg = String()
        status_msg.data = f"Mode: {self.control_mode}, Balance: {self.balance_active}"
        self.status_pub.publish(status_msg)

    def set_control_mode(self, mode):
        """Set the control mode"""
        if mode in ['idle', 'walk', 'balance', 'manual']:
            self.control_mode = mode
            self.get_logger().info(f'Switched to control mode: {mode}')

    def set_target_velocity(self, linear_x, angular_z):
        """Set target velocity for walking"""
        self.target_velocity.linear.x = linear_x
        self.target_velocity.angular.z = angular_z
        if linear_x != 0 or angular_z != 0:
            self.control_mode = 'walk'

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    # Example: Change control mode based on parameters or service calls
    # In a real system, this would be controlled by higher-level logic
    controller.set_control_mode('balance')

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Humanoid Controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Simple Agent for High-Level Decision Making

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_msgs.msg import Int8  # For state machine
import math

class HumanoidAgent(Node):
    def __init__(self):
        super().__init__('humanoid_agent')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(Int8, '/control_mode', 10)
        self.behavior_pub = self.create_publisher(String, '/behavior_status', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/robot_status', self.status_callback, 10
        )

        # Timer for decision making
        self.timer = self.create_timer(0.5, self.decision_loop)  # 2Hz decision making

        # Internal state
        self.scan_data = None
        self.imu_data = None
        self.robot_status = "unknown"
        self.current_behavior = "idle"
        self.safety_engaged = False

        # Behavior parameters
        self.safe_distance = 0.5  # meters
        self.max_pitch_angle = 0.2  # radians

        self.get_logger().info('Humanoid Agent initialized')

    def scan_callback(self, msg):
        """Callback for laser scan data"""
        self.scan_data = msg.ranges

    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.imu_data = msg

    def status_callback(self, msg):
        """Callback for robot status"""
        self.robot_status = msg.data

    def decision_loop(self):
        """Main decision making loop"""
        if self.safety_check():
            self.emergency_stop()
            return

        # Evaluate current situation and select behavior
        if self.is_obstacle_ahead():
            self.execute_avoidance_behavior()
        elif self.is_unstable():
            self.execute_stabilization_behavior()
        else:
            self.execute_exploration_behavior()

    def safety_check(self):
        """Check for safety violations"""
        if self.imu_data is None:
            return False  # Can't evaluate safety without IMU

        # Check for dangerous pitch/roll angles
        pitch = math.asin(2.0 * (self.imu_data.orientation.w * self.imu_data.orientation.y -
                                self.imu_data.orientation.z * self.imu_data.orientation.x))

        return abs(pitch) > self.max_pitch_angle

    def is_obstacle_ahead(self):
        """Check if there's an obstacle in front"""
        if not self.scan_data:
            return False

        # Check front 60 degrees
        front_ranges = self.scan_data[150:210]  # Assuming 360-degree scan
        if not front_ranges:
            return False

        min_distance = min([r for r in front_ranges if r > 0], default=float('inf'))
        return min_distance < self.safe_distance

    def is_unstable(self):
        """Check if robot is unstable based on IMU data"""
        if self.imu_data is None:
            return False

        # Check for excessive angular velocity or acceleration
        angular_vel_mag = math.sqrt(
            self.imu_data.angular_velocity.x**2 +
            self.imu_data.angular_velocity.y**2 +
            self.imu_data.angular_velocity.z**2
        )

        return angular_vel_mag > 1.0  # threshold in rad/s

    def execute_avoidance_behavior(self):
        """Execute obstacle avoidance behavior"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Turn right to avoid obstacle

        self.cmd_vel_pub.publish(cmd)
        self.current_behavior = "avoiding_obstacle"
        self.publish_behavior_status()

    def execute_stabilization_behavior(self):
        """Execute stabilization behavior"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0  # Stop all motion for stabilization

        self.cmd_vel_pub.publish(cmd)
        self.current_behavior = "stabilizing"
        self.publish_behavior_status()

        # Switch to balance mode
        mode_msg = Int8()
        mode_msg.data = 2  # Assuming 2 is balance mode
        self.mode_pub.publish(mode_msg)

    def execute_exploration_behavior(self):
        """Execute exploration behavior"""
        cmd = Twist()
        cmd.linear.x = 0.2  # Move forward slowly
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)
        self.current_behavior = "exploring"
        self.publish_behavior_status()

    def emergency_stop(self):
        """Execute emergency stop"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)
        self.current_behavior = "emergency_stop"
        self.publish_behavior_status()
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')

    def publish_behavior_status(self):
        """Publish current behavior status"""
        status_msg = String()
        status_msg.data = self.current_behavior
        self.behavior_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    agent = HumanoidAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Shutting down Humanoid Agent')
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File for the Complete System

```python
# launch/humanoid_complete_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get config file path if needed
    config = os.path.join(
        get_package_share_directory('my_humanoid_package'),
        'config',
        'humanoid_params.yaml'
    )

    return LaunchDescription([
        # Humanoid Controller Node
        Node(
            package='my_humanoid_package',
            executable='humanoid_controller',
            name='humanoid_controller',
            parameters=[
                {'loop_rate': 100},  # 100Hz control loop
                {'balance_kp': 5.0},
                {'balance_kd': 1.0},
            ],
            output='screen'
        ),

        # High-Level Agent Node
        Node(
            package='my_humanoid_package',
            executable='humanoid_agent',
            name='humanoid_agent',
            parameters=[
                {'safe_distance': 0.5},
                {'max_pitch_angle': 0.2},
            ],
            output='screen'
        ),

        # Additional nodes could be added here:
        # - Perception nodes
        # - Planning nodes
        # - Simulation interfaces
    ])
```

## Running the System

To run this complete humanoid system:

1. **Start the controller:**
```bash
ros2 run my_humanoid_package humanoid_controller
```

2. **Start the agent:**
```bash
ros2 run my_humanoid_package humanoid_agent
```

3. **Or launch the complete system:**
```bash
ros2 launch my_humanoid_package humanoid_complete_system.launch.py
```

4. **Monitor the system:**
```bash
# Check topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Monitor robot status
ros2 topic echo /robot_status
```

This complete example demonstrates how multiple rclpy nodes can work together to create a sophisticated humanoid robot control system, with proper separation of concerns between low-level control and high-level decision making.