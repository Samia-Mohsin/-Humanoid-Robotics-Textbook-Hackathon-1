---
sidebar_position: 13
---

# Time Synchronization Mechanisms

Time synchronization is critical for maintaining consistency between Gazebo physics simulation and Unity visualization. Without proper time synchronization, the two platforms will drift apart, leading to visual artifacts and inconsistent behavior.

## Understanding Time Synchronization Challenges

### Clock Drift
- Different systems may have slightly different clock speeds
- Process scheduling can cause timing variations
- System load affects computation time

### Latency Effects
- Network delays can cause time discrepancies
- Processing delays add to synchronization errors
- Variable delays make synchronization more complex

### Simulation vs. Real-time
- Gazebo may run faster or slower than real-time
- Unity rendering frame rate may vary
- Need to account for different timing models

## Time Synchronization Approaches

### 1. Master Clock Synchronization

Using one platform as the time reference:

```python
# Master Clock Publisher (Gazebo)
import rospy
from std_msgs.msg import Float64
from time import time

class MasterClock:
    def __init__(self):
        rospy.init_node('master_clock')
        self.time_pub = rospy.Publisher('/synchronization_time', Float64, queue_size=10)

        # Reference time (simulation time or wall clock)
        self.start_time = time()
        self.sim_start_time = rospy.Time.now().to_sec()

    def publish_time(self):
        rate = rospy.Rate(100)  # 100 Hz

        while not rospy.is_shutdown():
            # Calculate synchronized time
            current_time = rospy.get_rostime().to_sec() - self.sim_start_time
            time_msg = Float64()
            time_msg.data = current_time

            self.time_pub.publish(time_msg)
            rate.sleep()
```

```csharp
// Slave Clock Subscriber (Unity)
using UnityEngine;
using RosSharp;

public class SlaveClock : MonoBehaviour
{
    [SerializeField] private float maxDrift = 0.1f; // Maximum acceptable drift in seconds

    private double referenceTime = 0.0f;
    private double localTimeOffset = 0.0f;
    private double lastSyncTime = 0.0f;

    private RosSocket rosSocket;
    private string timeTopic = "/synchronization_time";

    void Start()
    {
        rosSocket = new RosSocket("ws://localhost:9090");
        rosSocket.Subscribe<std_msgs.Float64>(timeTopic, OnTimeReceived);
    }

    private void OnTimeReceived(std_msgs.Float64 timeMsg)
    {
        double currentTime = Time.time;
        double receivedTime = timeMsg.data;

        // Calculate offset to maintain synchronization
        double expectedLocalTime = receivedTime - localTimeOffset;
        double drift = currentTime - expectedLocalTime;

        // Apply correction if drift is within acceptable range
        if (Mathf.Abs((float)drift) < maxDrift)
        {
            localTimeOffset += drift * 0.1f; // Gradual correction
        }

        referenceTime = receivedTime;
        lastSyncTime = currentTime;
    }

    public double GetSynchronizedTime()
    {
        // Apply offset to current time
        return Time.time - localTimeOffset;
    }
}
```

### 2. NTP-Style Synchronization

Implementing a custom NTP-like protocol for tighter synchronization:

```python
# Time Synchronization Client (Gazebo)
import socket
import time
import struct

class NTPSyncClient:
    def __init__(self, server_host='localhost', server_port=12345):
        self.server_host = server_host
        self.server_port = server_port
        self.offset = 0.0
        self.delay = 0.0

    def synchronize_time(self):
        # Send timestamp request
        client_time1 = time.time()

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(5.0)

            # Send request packet
            request = struct.pack('d', client_time1)
            sock.sendto(request, (self.server_host, self.server_port))

            # Receive response
            response, _ = sock.recvfrom(256)
            server_time, client_time2 = struct.unpack('dd', response)

            # Calculate round-trip delay and offset
            client_time2 = time.time()
            self.delay = (client_time2 - client_time1) - (server_time - client_time1)
            self.offset = ((server_time - client_time1) + (server_time - client_time2)) / 2.0

            return self.offset, self.delay

    def get_corrected_time(self):
        return time.time() + self.offset
```

```csharp
// Time Synchronization Server (Unity)
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class NTPSyncServer : MonoBehaviour
{
    private UdpClient udpServer;
    private Thread serverThread;
    private bool running = false;

    [SerializeField] private int port = 12345;

    void Start()
    {
        StartServer();
    }

    private void StartServer()
    {
        udpServer = new UdpClient(port);
        running = true;

        serverThread = new Thread(HandleRequests);
        serverThread.Start();
    }

    private void HandleRequests()
    {
        IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);

        while (running)
        {
            try
            {
                byte[] data = udpServer.Receive(ref remoteEP);
                double clientTime = System.BitConverter.ToDouble(data, 0);

                // Send response with server time and round-trip info
                double serverTime = Time.time;
                double clientTime2 = clientTime; // In real implementation, this would be when we received the request

                byte[] response = new byte[16];
                System.Buffer.BlockCopy(System.BitConverter.GetBytes(serverTime), 0, response, 0, 8);
                System.Buffer.BlockCopy(System.BitConverter.GetBytes(clientTime2), 0, response, 8, 8);

                udpServer.Send(response, response.Length, remoteEP);
            }
            catch (System.Exception e)
            {
                if (running)
                    Debug.LogError("Time sync server error: " + e.Message);
            }
        }
    }
}
```

### 3. Interpolation-Based Synchronization

Using interpolation to smooth out timing differences:

```python
# Interpolation-Based Synchronization (Gazebo)
import rospy
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class InterpolatedSync:
    def __init__(self):
        rospy.init_node('interpolated_sync')

        # Store previous and current states for interpolation
        self.previous_state = {}
        self.current_state = {}
        self.last_update_time = time.time()

        # Publisher for interpolated poses
        self.pose_pub = rospy.Publisher('/interpolated_poses', Pose, queue_size=10)

    def update_state(self, new_state):
        # Store previous state and update current
        self.previous_state = self.current_state.copy()
        self.current_state = new_state.copy()
        self.last_update_time = time.time()

    def interpolate_to_time(self, target_time):
        current_time = time.time()
        time_diff = current_time - self.last_update_time

        # Interpolation factor (0 = previous state, 1 = current state)
        factor = min(1.0, time_diff / 0.1)  # Complete interpolation in 100ms

        interpolated_state = {}
        for obj_name, current_pose in self.current_state.items():
            if obj_name in self.previous_state:
                prev_pose = self.previous_state[obj_name]

                # Linear interpolation for position
                interp_pose = Pose()
                interp_pose.position.x = prev_pose.position.x + \
                    (current_pose.position.x - prev_pose.position.x) * factor
                interp_pose.position.y = prev_pose.position.y + \
                    (current_pose.position.y - prev_pose.position.y) * factor
                interp_pose.position.z = prev_pose.position.z + \
                    (current_pose.position.z - prev_pose.position.z) * factor

                # Slerp for orientation
                # In practice, use quaternion slerp here
                interp_pose.orientation = current_pose.orientation

                interpolated_state[obj_name] = interp_pose
            else:
                interpolated_state[obj_name] = current_pose

        return interpolated_state
```

```csharp
// Interpolation-Based Synchronization (Unity)
using UnityEngine;

public class InterpolatedTimeSync : MonoBehaviour
{
    [System.Serializable]
    public class ObjectState
    {
        public string name;
        public Vector3 position;
        public Quaternion rotation;
        public float timestamp;
    }

    public float interpolationPeriod = 0.1f; // 100ms interpolation period
    private ObjectState[] previousStates;
    private ObjectState[] currentStates;
    private float lastUpdateTime;

    private System.Collections.Generic.Dictionary<string, Transform> objectMap;

    void Start()
    {
        objectMap = new System.Collections.Generic.Dictionary<string, Transform>();

        // Populate object map with all objects that need synchronization
        foreach (Transform child in transform)
        {
            objectMap[child.name] = child;
        }

        InitializeStates();
    }

    void InitializeStates()
    {
        // Initialize state arrays based on the objects we're tracking
        currentStates = new ObjectState[objectMap.Count];
        previousStates = new ObjectState[objectMap.Count];

        int i = 0;
        foreach (var pair in objectMap)
        {
            currentStates[i] = new ObjectState
            {
                name = pair.Key,
                position = pair.Value.position,
                rotation = pair.Value.rotation,
                timestamp = Time.time
            };
            previousStates[i] = currentStates[i];
            i++;
        }
        lastUpdateTime = Time.time;
    }

    public void UpdateObjectState(string objectName, Vector3 position, Quaternion rotation)
    {
        // Update current state
        for (int i = 0; i < currentStates.Length; i++)
        {
            if (currentStates[i].name == objectName)
            {
                // Move current state to previous
                previousStates[i] = currentStates[i];

                // Update current state
                currentStates[i] = new ObjectState
                {
                    name = objectName,
                    position = position,
                    rotation = rotation,
                    timestamp = Time.time
                };
                lastUpdateTime = Time.time;
                break;
            }
        }
    }

    void Update()
    {
        // Apply interpolated positions to objects
        float interpolationFactor = GetInterpolationFactor();

        foreach (var state in currentStates)
        {
            Transform objTransform;
            if (objectMap.TryGetValue(state.name, out objTransform))
            {
                ObjectState prevState;
                // Find matching previous state
                foreach (var prev in previousStates)
                {
                    if (prev.name == state.name)
                    {
                        prevState = prev;

                        // Apply interpolation
                        Vector3 interpolatedPos = Vector3.Lerp(
                            prevState.position,
                            state.position,
                            interpolationFactor
                        );

                        Quaternion interpolatedRot = Quaternion.Slerp(
                            prevState.rotation,
                            state.rotation,
                            interpolationFactor
                        );

                        objTransform.position = interpolatedPos;
                        objTransform.rotation = interpolatedRot;
                        break;
                    }
                }
            }
        }
    }

    private float GetInterpolationFactor()
    {
        float currentTime = Time.time;
        float timeDiff = currentTime - lastUpdateTime;
        return Mathf.Clamp01(timeDiff / interpolationPeriod);
    }
}
```

## Advanced Time Synchronization Techniques

### 1. Adaptive Synchronization

Adjust synchronization parameters based on observed performance:

```python
# Adaptive Time Synchronization
class AdaptiveSync:
    def __init__(self):
        self.sync_history = []
        self.max_history = 100
        self.adjustment_factor = 0.1

    def add_sync_measurement(self, drift):
        self.sync_history.append(drift)
        if len(self.sync_history) > self.max_history:
            self.sync_history.pop(0)

    def calculate_adjustment(self):
        if len(self.sync_history) < 10:
            return self.adjustment_factor

        # Calculate average drift and variance
        avg_drift = sum(self.sync_history) / len(self.sync_history)
        variance = sum((x - avg_drift) ** 2 for x in self.sync_history) / len(self.sync_history)

        # Adjust factor based on stability
        if variance < 0.001:  # Stable
            self.adjustment_factor = max(0.01, self.adjustment_factor * 0.95)
        else:  # Unstable
            self.adjustment_factor = min(0.5, self.adjustment_factor * 1.1)

        return self.adjustment_factor
```

### 2. Predictive Synchronization

Predict future states to compensate for communication delays:

```python
# Predictive Synchronization
import numpy as np

class PredictiveSync:
    def __init__(self):
        self.state_history = {}  # Store recent states for prediction

    def add_state_measurement(self, obj_name, position, velocity, timestamp):
        if obj_name not in self.state_history:
            self.state_history[obj_name] = []

        self.state_history[obj_name].append((position, velocity, timestamp))

        # Keep only recent measurements
        if len(self.state_history[obj_name]) > 10:
            self.state_history[obj_name].pop(0)

    def predict_position(self, obj_name, future_time):
        if obj_name not in self.state_history or len(self.state_history[obj_name]) < 2:
            return None

        # Use last two measurements for simple prediction
        prev_pos, prev_vel, prev_time = self.state_history[obj_name][-2]
        curr_pos, curr_vel, curr_time = self.state_history[obj_name][-1]

        # Predict based on current velocity
        time_diff = future_time - curr_time
        predicted_pos = curr_pos + curr_vel * time_diff

        return predicted_pos
```

## Best Practices

1. **Monitor Synchronization Quality**: Continuously track drift and adjust parameters
2. **Handle Network Partitions**: Implement recovery when connections are restored
3. **Use Multiple Reference Points**: Cross-check synchronization using multiple metrics
4. **Log Synchronization Events**: Maintain logs for debugging and optimization
5. **Test Under Load**: Verify synchronization works under various system loads
6. **Implement Graceful Degradation**: Maintain functionality even with imperfect sync

## Troubleshooting

### Common Issues
- **Clock Drift**: Implement periodic resynchronization
- **Network Jitter**: Use interpolation to smooth out variable delays
- **Buffer Underruns**: Maintain sufficient prediction buffers
- **System Load Variations**: Monitor and adapt to changing performance

### Performance Monitoring
- Track synchronization error over time
- Monitor communication latency
- Measure timing variance
- Log synchronization corrections applied

These time synchronization mechanisms ensure that both Gazebo physics and Unity visualization operate on a consistent time base, which is essential for creating seamless and believable digital twin experiences.