---
sidebar_position: 2
---

# Nodes, Topics, and Services in ROS 2

## Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. In humanoid robotics, nodes typically represent different functional components such as sensor drivers, control algorithms, perception systems, and planning modules.

### Creating a Node

```python
import rclpy
from rclpy.node import Node

class HumanoidControllerNode(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid Controller node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidControllerNode()

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

### Node Lifecycle
Nodes in ROS 2 have a well-defined lifecycle that includes initialization, activation, and cleanup phases, which is particularly important for humanoid robots where graceful startup and shutdown are critical for safety.

## Topics - Publisher/Subscriber Pattern

Topics enable asynchronous communication through a publish-subscribe model. This is ideal for sensor data, state updates, and other continuous streams of information in humanoid robots.

### Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.01  # seconds (100Hz for humanoid control)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = [0.0, 0.1, -0.05]  # Example positions
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_publisher.destroy_node()
        rclpy.shutdown()
```

### Creating a Subscriber

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint states: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_subscriber = JointStateSubscriber()

    try:
        rclpy.spin(joint_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_subscriber.destroy_node()
        rclpy.shutdown()
```

## Services - Request/Response Pattern

Services provide synchronous communication where a client sends a request and waits for a response. This is useful for configuration changes, one-time calculations, or operations that must complete before proceeding.

### Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class BalanceService(Node):
    def __init__(self):
        super().__init__('balance_service')
        self.srv = self.create_service(
            SetBool,
            'enable_balance_control',
            self.enable_balance_callback
        )
        self.balance_enabled = False

    def enable_balance_callback(self, request, response):
        self.balance_enabled = request.data
        response.success = True
        response.message = f'Balance control {"enabled" if self.balance_enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    balance_service = BalanceService()

    try:
        rclpy.spin(balance_service)
    except KeyboardInterrupt:
        pass
    finally:
        balance_service.destroy_node()
        rclpy.shutdown()
```

### Creating a Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class BalanceClient(Node):
    def __init__(self):
        super().__init__('balance_client')
        self.cli = self.create_client(SetBool, 'enable_balance_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, enable_balance):
        self.req.data = enable_balance
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    balance_client = BalanceClient()

    response = balance_client.send_request(True)
    if response is not None:
        print(f'Result: {response.message}')
    else:
        print('Service call failed')

    balance_client.destroy_node()
    rclpy.shutdown()
```

## Quality of Service (QoS) Considerations for Humanoid Robotics

Different types of data in humanoid robots require different QoS settings:

### For Critical Control Data
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

critical_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

### For Sensor Data
```python
sensor_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

## Best Practices for Humanoid Robotics

1. **Use appropriate message types**: Leverage standard ROS 2 message types (sensor_msgs, geometry_msgs, etc.) for interoperability
2. **Consider timing requirements**: Humanoid robots often need high-frequency communication (100Hz+ for control)
3. **Handle node failures gracefully**: Implement proper error handling and recovery mechanisms
4. **Use namespaces**: Organize nodes and topics with appropriate namespaces for multi-robot systems
5. **Monitor communication**: Use ROS 2 tools to monitor topic rates and detect communication issues

## Common Patterns in Humanoid Robotics

### Sensor Integration Pattern
Multiple sensors publish to a common topic for sensor fusion.

### Control Hierarchy Pattern
High-level planners publish commands, low-level controllers subscribe and execute them.

### State Monitoring Pattern
Various subsystems publish their state to monitoring nodes for system health assessment.