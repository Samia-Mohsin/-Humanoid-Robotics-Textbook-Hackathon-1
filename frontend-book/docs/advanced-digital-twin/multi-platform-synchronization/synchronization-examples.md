---
sidebar_position: 16
---

# Synchronization Examples

This page provides practical examples demonstrating synchronization implementations that achieve &lt;50ms latency between Gazebo and Unity. These examples showcase real-world approaches to multi-platform synchronization.

## Example 1: Basic Low-Latency Synchronization

A complete example showing the minimum viable implementation for sub-50ms synchronization:

### Gazebo Publisher (Python)

```python
#!/usr/bin/env python3

import rospy
import time
import json
import socket
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class LowLatencySync:
    def __init__(self):
        rospy.init_node('low_latency_sync')

        # UDP socket for fast communication
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setblocking(False)  # Non-blocking
        self.unity_ip = "127.0.0.1"
        self.unity_port = 8888

        # Optimized publishers
        self.rate = rospy.Rate(200)  # 200 Hz for low latency
        self.frame_count = 0

    def get_optimized_state_data(self):
        """Get essential state data in compact format"""
        # In practice, this would interface with Gazebo
        # For this example, we'll simulate data
        timestamp = time.time()

        # Compact representation - only send essential data
        objects_data = [
            {
                "n": "robot::base_link",  # Short name
                "p": [1.0, 2.0, 3.0],    # Position [x, y, z]
                "r": [0.0, 0.0, 0.0, 1.0] # Rotation quaternion [x, y, z, w]
            },
            {
                "n": "robot::head",
                "p": [1.1, 2.1, 3.5],
                "r": [0.1, 0.0, 0.0, 0.995]
            }
        ]

        return {
            "t": timestamp,  # Timestamp
            "f": self.frame_count,  # Frame count
            "o": objects_data  # Objects
        }

    def run(self):
        """Main synchronization loop"""
        while not rospy.is_shutdown():
            start_time = time.time()

            # Get optimized state data
            state_data = self.get_optimized_state_data()

            # Serialize to compact JSON
            json_data = json.dumps(state_data, separators=(',', ':'))

            # Send via UDP (fastest transport)
            try:
                self.udp_socket.sendto(
                    json_data.encode('utf-8'),
                    (self.unity_ip, self.unity_port)
                )
            except socket.error as e:
                rospy.logwarn(f"UDP send failed: {e}")

            self.frame_count += 1

            # Calculate processing time
            processing_time = (time.time() - start_time) * 1000  # ms
            rospy.logdebug(f"Processing time: {processing_time:.2f}ms")

            # Maintain 200Hz rate
            self.rate.sleep()

if __name__ == '__main__':
    sync = LowLatencySync()
    sync.run()
```

### Unity Subscriber (C#)

```csharp
using UnityEngine;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Concurrent;

public class LowLatencySyncSubscriber : MonoBehaviour
{
    [System.Serializable]
    public class SyncObject
    {
        public string gazeboName;
        public GameObject unityObject;
    }

    [SerializeField] private SyncObject[] syncObjects;
    private UdpClient udpClient;
    private Thread receiveThread;
    private bool running = false;

    // Interpolation for smooth movement
    private System.Collections.Generic.Dictionary<string, InterpolationData> interpolationData;

    [System.Serializable]
    private class InterpolationData
    {
        public Vector3 targetPosition;
        public Quaternion targetRotation;
        public Vector3 currentVelocity;
        public float lastUpdateTime;
        public bool hasData;
    }

    void Start()
    {
        StartUDPListener();
        InitializeInterpolationData();
    }

    private void StartUDPListener()
    {
        try
        {
            udpClient = new UdpClient(8888);
            udpClient.Client.ReceiveBufferSize = 65536; // Large buffer
            running = true;

            receiveThread = new Thread(ReceiveData);
            receiveThread.IsBackground = true;
            receiveThread.Start();
        }
        catch (System.Exception e)
        {
            Debug.LogError("Failed to start UDP listener: " + e.Message);
        }
    }

    private void InitializeInterpolationData()
    {
        interpolationData = new System.Collections.Generic.Dictionary<string, InterpolationData>();

        foreach (var syncObj in syncObjects)
        {
            if (syncObj.unityObject != null)
            {
                interpolationData[syncObj.gazeboName] = new InterpolationData
                {
                    targetPosition = syncObj.unityObject.transform.position,
                    targetRotation = syncObj.unityObject.transform.rotation,
                    currentVelocity = Vector3.zero,
                    lastUpdateTime = Time.time,
                    hasData = true
                };
            }
        }
    }

    private void ReceiveData()
    {
        System.Net.IPEndPoint remoteEP = new System.Net.IPEndPoint(
            System.Net.IPAddress.Any, 0
        );

        byte[] buffer = new byte[8192]; // Large buffer

        while (running)
        {
            try
            {
                int receivedBytes = udpClient.Client.Receive(buffer, 0, buffer.Length,
                    SocketFlags.None);

                if (receivedBytes > 0)
                {
                    string jsonData = System.Text.Encoding.UTF8.GetString(
                        buffer, 0, receivedBytes
                    );

                    // Process on main thread
                    UnityEngine.WSA.Application.InvokeOnAppThread(() => {
                        ProcessReceivedData(jsonData);
                    }, false);
                }
            }
            catch (System.Exception e)
            {
                if (running)
                    Debug.LogError("UDP receive error: " + e.Message);
                break;
            }
        }
    }

    private void ProcessReceivedData(string jsonData)
    {
        try
        {
            // Simple JSON parsing (in practice, use a proper JSON library)
            var stateData = SimpleJSON.JSON.Parse(jsonData);

            // Process each object
            var objectsArray = stateData["o"].AsArray;
            foreach (var objData in objectsArray)
            {
                string name = objData["n"];
                var positionArray = objData["p"].AsArray;
                var rotationArray = objData["r"].AsArray;

                Vector3 newPosition = new Vector3(
                    positionArray[1], // Convert Gazebo Y to Unity X
                    positionArray[2], // Convert Gazebo Z to Unity Y
                    positionArray[0]  // Convert Gazebo X to Unity Z
                );

                Quaternion newRotation = new Quaternion(
                    rotationArray[1], // Convert Gazebo Y to Unity X
                    rotationArray[2], // Convert Gazebo Z to Unity Y
                    rotationArray[0], // Convert Gazebo X to Unity Z
                    -rotationArray[3] // Convert Gazebo W with sign flip
                );

                // Update interpolation data
                if (interpolationData.ContainsKey(name))
                {
                    var interpData = interpolationData[name];
                    interpData.targetPosition = newPosition;
                    interpData.targetRotation = newRotation;
                    interpData.lastUpdateTime = Time.time;
                    interpData.hasData = true;
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error parsing sync data: " + e.Message);
        }
    }

    void Update()
    {
        // Apply interpolated positions for smooth movement
        float interpolationDelay = 0.02f; // 20ms delay for network jitter
        float currentTime = Time.time - interpolationDelay;

        foreach (var syncObj in syncObjects)
        {
            if (syncObj.unityObject != null &&
                interpolationData.ContainsKey(syncObj.gazeboName))
            {
                var interpData = interpolationData[syncObj.gazeboName];

                if (interpData.hasData)
                {
                    // Apply smooth interpolation
                    syncObj.unityObject.transform.position = Vector3.Lerp(
                        syncObj.unityObject.transform.position,
                        interpData.targetPosition,
                        Time.deltaTime * 15f // Smooth factor
                    );

                    syncObj.unityObject.transform.rotation = Quaternion.Slerp(
                        syncObj.unityObject.transform.rotation,
                        interpData.targetRotation,
                        Time.deltaTime * 15f // Smooth factor
                    );
                }
            }
        }
    }

    void OnApplicationQuit()
    {
        running = false;
        if (udpClient != null)
        {
            udpClient.Close();
        }
        if (receiveThread != null)
        {
            receiveThread.Join();
        }
    }
}

// Simple JSON parser for Unity (minimal implementation)
public class SimpleJSON
{
    public class JSONNode
    {
        public System.Collections.Generic.Dictionary<string, JSONNode> dict;
        public System.Collections.Generic.List<JSONNode> array;

        public JSONNode this[int index] => array[index];
        public JSONNode this[string key] => dict[key];

        public System.Collections.Generic.List<JSONNode> AsArray => array;
        public float AsFloat => float.Parse(ToString());
        public string AsString => ToString();

        public override string ToString()
        {
            // Simplified - in practice use a proper JSON library
            return "";
        }
    }

    public static JSONNode Parse(string json)
    {
        // Simplified parser - in practice use a proper JSON library
        var node = new JSONNode();
        node.dict = new System.Collections.Generic.Dictionary<string, JSONNode>();
        node.array = new System.Collections.Generic.List<JSONNode>();
        return node;
    }
}
```

## Example 2: Advanced Synchronization with Performance Monitoring

A more sophisticated example that includes performance monitoring and adaptive behavior:

### Gazebo Performance-Monitored Publisher

```python
#!/usr/bin/env python3

import rospy
import time
import struct
import socket
from collections import deque
import threading

class PerformanceMonitoredSync:
    def __init__(self):
        rospy.init_node('perf_monitored_sync')

        # UDP socket setup
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        self.unity_ip = "127.0.0.1"
        self.unity_port = 8889

        # Performance monitoring
        self.latency_samples = deque(maxlen=100)
        self.processing_times = deque(maxlen=100)
        self.message_sizes = deque(maxlen=100)

        # Adaptive rate control
        self.target_latency = 40.0  # ms
        self.current_rate = 150  # Hz
        self.min_rate = 50
        self.max_rate = 300
        self.rate_adjustment = 0

        # Threading for non-blocking operations
        self.publish_lock = threading.Lock()
        self.running = True

    def measure_performance(self, start_time, message_size):
        """Measure and record performance metrics"""
        processing_time = (time.time() - start_time) * 1000  # ms
        self.processing_times.append(processing_time)
        self.message_sizes.append(message_size)

        # Calculate average processing time
        avg_processing = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0

        # Adjust rate based on performance
        if avg_processing > 5.0:  # If processing takes too long
            self.current_rate = max(self.min_rate, self.current_rate * 0.95)
        elif avg_processing < 2.0:  # If processing is fast
            self.current_rate = min(self.max_rate, self.current_rate * 1.02)

        return processing_time

    def create_binary_message(self, obj_data):
        """Create compact binary message"""
        # Format: [timestamp(8bytes), seq_num(4bytes), obj_count(2bytes),
        # [name_len(1byte), name, pos(12bytes), rot(16bytes)]*n]

        message = bytearray()

        # Add timestamp and sequence number
        message.extend(struct.pack('dI', time.time(), obj_data.get('seq', 0)))

        # Add object count
        objects = obj_data.get('objects', [])
        message.extend(struct.pack('H', len(objects)))

        # Add each object
        for obj in objects:
            name = obj['name'].encode('utf-8')
            message.extend(struct.pack('B', len(name)))  # Name length
            message.extend(name)  # Name

            # Position (x, y, z) - 3 floats = 12 bytes
            pos = obj['position']
            message.extend(struct.pack('fff', pos[0], pos[1], pos[2]))

            # Rotation (x, y, z, w) - 4 floats = 16 bytes
            rot = obj['rotation']
            message.extend(struct.pack('ffff', rot[0], rot[1], rot[2], rot[3]))

        return message

    def run(self):
        """Main performance-optimized loop"""
        seq_num = 0

        while not rospy.is_shutdown() and self.running:
            start_time = time.time()

            # Create optimized object data
            obj_data = {
                'seq': seq_num,
                'objects': [
                    {
                        'name': 'robot::base',
                        'position': [1.0, 2.0, 3.0],
                        'rotation': [0.0, 0.0, 0.0, 1.0]
                    },
                    {
                        'name': 'robot::head',
                        'position': [1.1, 2.1, 3.5],
                        'rotation': [0.1, 0.0, 0.0, 0.995]
                    }
                ]
            }

            # Create binary message
            message = self.create_binary_message(obj_data)
            message_size = len(message)

            # Send via UDP
            try:
                self.udp_socket.sendto(message, (self.unity_ip, self.unity_port))
            except socket.error as e:
                rospy.logwarn(f"Send failed: {e}")

            # Measure performance
            processing_time = self.measure_performance(start_time, message_size)

            # Log performance if needed
            if seq_num % 100 == 0:  # Log every 100 messages
                rospy.loginfo(f"Seq: {seq_num}, Rate: {self.current_rate:.1f}Hz, "
                             f"Proc: {processing_time:.2f}ms, Size: {message_size}B")

            seq_num += 1

            # Sleep based on current adaptive rate
            sleep_time = 1.0 / self.current_rate
            time.sleep(max(0.001, sleep_time))  # Minimum 1ms sleep

if __name__ == '__main__':
    sync = PerformanceMonitoredSync()
    try:
        sync.run()
    except KeyboardInterrupt:
        sync.running = False
        print("Performance monitored sync stopped.")
```

## Example 3: Unity Performance-Monitored Subscriber

```csharp
using UnityEngine;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;

public class PerformanceMonitoredSubscriber : MonoBehaviour
{
    [System.Serializable]
    public class TrackedObject
    {
        public string gazeboName;
        public GameObject unityObject;
    }

    [SerializeField] private TrackedObject[] trackedObjects;
    private UdpClient udpClient;
    private Thread receiveThread;
    private bool running = false;

    // Performance monitoring
    private Queue<float> latencyHistory = new Queue<float>();
    private Queue<float> processingTimes = new Queue<float>();
    private const int MAX_SAMPLES = 200;

    // Interpolation with adaptive parameters
    private Dictionary<string, ObjectState> objectStates = new Dictionary<string, ObjectState>();

    [System.Serializable]
    private class ObjectState
    {
        public Vector3 targetPosition;
        public Quaternion targetRotation;
        public Vector3 lastPosition;
        public Quaternion lastRotation;
        public float lastReceiveTime;
        public bool hasData;
    }

    [Header("Performance Settings")]
    [SerializeField] private float targetLatency = 40f; // ms
    [SerializeField] private float interpolationSpeed = 15f;
    [SerializeField] private float maxInterpolationDelay = 50f; // ms

    void Start()
    {
        StartSynchronizedSubscriber();
        InitializeObjectStates();
    }

    private void StartSynchronizedSubscriber()
    {
        try
        {
            udpClient = new UdpClient(8889);
            udpClient.Client.ReceiveBufferSize = 131072; // Large receive buffer
            running = true;

            receiveThread = new Thread(ReceiveDataLoop);
            receiveThread.IsBackground = true;
            receiveThread.Start();
        }
        catch (System.Exception e)
        {
            Debug.LogError("Failed to start UDP listener: " + e.Message);
        }
    }

    private void InitializeObjectStates()
    {
        foreach (var trackedObj in trackedObjects)
        {
            if (trackedObj.unityObject != null)
            {
                objectStates[trackedObj.gazeboName] = new ObjectState
                {
                    targetPosition = trackedObj.unityObject.transform.position,
                    targetRotation = trackedObj.unityObject.transform.rotation,
                    lastPosition = trackedObj.unityObject.transform.position,
                    lastRotation = trackedObj.unityObject.transform.rotation,
                    lastReceiveTime = Time.time,
                    hasData = false
                };
            }
        }
    }

    private void ReceiveDataLoop()
    {
        System.Net.IPEndPoint remoteEP = new System.Net.IPEndPoint(
            System.Net.IPAddress.Any, 0
        );

        byte[] buffer = new byte[16384]; // Large buffer for binary data

        while (running)
        {
            try
            {
                int receivedBytes = udpClient.Client.Receive(buffer, 0, buffer.Length,
                    SocketFlags.None);

                if (receivedBytes > 0)
                {
                    var receiveTime = Time.realtimeSinceStartup;

                    // Process received binary data
                    ProcessBinaryData(buffer, receivedBytes, receiveTime);
                }
            }
            catch (System.Exception e)
            {
                if (running)
                    Debug.LogError("UDP receive error: " + e.Message);
                break;
            }
        }
    }

    private void ProcessBinaryData(byte[] data, int length, float receiveTime)
    {
        try
        {
            using (var stream = new System.IO.MemoryStream(data, 0, length))
            using (var reader = new System.IO.BinaryReader(stream))
            {
                // Read timestamp (8 bytes) and sequence number (4 bytes)
                double timestamp = reader.ReadDouble();
                uint seqNum = reader.ReadUInt32();

                // Calculate network latency (simplified)
                float networkLatency = (receiveTime - (float)timestamp) * 1000f; // ms
                AddLatencySample(networkLatency);

                // Read object count (2 bytes)
                ushort objCount = reader.ReadUInt16();

                // Process each object
                for (int i = 0; i < objCount; i++)
                {
                    // Read name length (1 byte) and name
                    byte nameLength = reader.ReadByte();
                    string name = new string(reader.ReadChars(nameLength));

                    // Read position (3 floats = 12 bytes)
                    float posX = reader.ReadSingle();
                    float posY = reader.ReadSingle();
                    float posZ = reader.ReadSingle();

                    // Read rotation (4 floats = 16 bytes)
                    float rotX = reader.ReadSingle();
                    float rotY = reader.ReadSingle();
                    float rotZ = reader.ReadSingle();
                    float rotW = reader.ReadSingle();

                    // Convert coordinates (Gazebo to Unity)
                    Vector3 position = new Vector3(posY, posZ, posX);
                    Quaternion rotation = new Quaternion(rotY, rotZ, rotX, -rotW);

                    // Update object state
                    if (objectStates.ContainsKey(name))
                    {
                        var state = objectStates[name];
                        state.lastPosition = state.targetPosition;
                        state.lastRotation = state.targetRotation;
                        state.targetPosition = position;
                        state.targetRotation = rotation;
                        state.lastReceiveTime = receiveTime;
                        state.hasData = true;
                    }
                }

                // Record processing time
                float processingTime = (Time.realtimeSinceStartup - receiveTime) * 1000f;
                AddProcessingTime(processingTime);
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error processing binary data: " + e.Message);
        }
    }

    private void AddLatencySample(float latency)
    {
        latencyHistory.Enqueue(latency);
        if (latencyHistory.Count > MAX_SAMPLES)
            latencyHistory.Dequeue();
    }

    private void AddProcessingTime(float time)
    {
        processingTimes.Enqueue(time);
        if (processingTimes.Count > MAX_SAMPLES)
            processingTimes.Dequeue();
    }

    void Update()
    {
        float currentTime = Time.realtimeSinceStartup;

        // Update all tracked objects with interpolation
        foreach (var trackedObj in trackedObjects)
        {
            if (trackedObj.unityObject != null &&
                objectStates.ContainsKey(trackedObj.gazeboName))
            {
                var state = objectStates[trackedObj.gazeboName];

                if (state.hasData)
                {
                    // Calculate adaptive interpolation factor based on latency
                    float timeDiff = currentTime - state.lastReceiveTime;
                    float adaptiveSpeed = CalculateAdaptiveSpeed(timeDiff);

                    // Apply smooth interpolation
                    trackedObj.unityObject.transform.position = Vector3.Lerp(
                        trackedObj.unityObject.transform.position,
                        state.targetPosition,
                        Time.deltaTime * adaptiveSpeed
                    );

                    trackedObj.unityObject.transform.rotation = Quaternion.Slerp(
                        trackedObj.unityObject.transform.rotation,
                        state.targetRotation,
                        Time.deltaTime * adaptiveSpeed
                    );
                }
            }
        }

        // Display performance metrics periodically
        if (Time.frameCount % 60 == 0) // Every 60 frames (roughly 1 sec at 60 FPS)
        {
            DisplayPerformanceMetrics();
        }
    }

    private float CalculateAdaptiveSpeed(float timeSinceUpdate)
    {
        // Adjust interpolation speed based on time since last update
        // If we haven't received updates in a while, interpolate faster
        if (timeSinceUpdate > 0.1f) // 100ms without update
            return interpolationSpeed * 2f; // Speed up interpolation
        else if (timeSinceUpdate < 0.005f) // 5ms since update
            return interpolationSpeed * 0.5f; // Slow down to be more precise
        else
            return interpolationSpeed;
    }

    private void DisplayPerformanceMetrics()
    {
        if (latencyHistory.Count > 0)
        {
            float avgLatency = 0;
            foreach (float latency in latencyHistory)
                avgLatency += latency;
            avgLatency /= latencyHistory.Count;

            float avgProcessing = 0;
            if (processingTimes.Count > 0)
            {
                foreach (float proc in processingTimes)
                    avgProcessing += proc;
                avgProcessing /= processingTimes.Count;
            }

            Debug.Log($"Sync Performance - Avg Latency: {avgLatency:F2}ms, " +
                     $"Avg Processing: {avgProcessing:F3}ms, " +
                     $"Objects: {objectStates.Count}");
        }
    }

    void OnApplicationQuit()
    {
        running = false;
        if (udpClient != null)
        {
            udpClient.Close();
        }
        if (receiveThread != null)
        {
            receiveThread.Join();
        }
    }
}
```

## Example 4: Troubleshooting and Optimization Tools

A set of tools to help diagnose and fix synchronization issues:

### Latency Measurement Tool

```python
# Latency measurement between Gazebo and Unity
import time
import socket
import threading
import statistics

class LatencyTester:
    def __init__(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind(('localhost', 9999))
        self.server_socket.settimeout(1.0)

        self.latency_samples = []
        self.running = False

    def start_test(self, duration=10):
        """Start latency measurement test"""
        self.running = True
        sender_thread = threading.Thread(target=self.send_ping)
        receiver_thread = threading.Thread(target=self.receive_pong)

        sender_thread.start()
        receiver_thread.start()

        time.sleep(duration)
        self.running = False

        sender_thread.join()
        receiver_thread.join()

        return self.analyze_results()

    def send_ping(self):
        """Send ping messages"""
        while self.running:
            timestamp = time.time()
            message = f"PING:{timestamp}".encode('utf-8')
            self.client_socket.sendto(message, ('localhost', 9999))
            time.sleep(0.05)  # 20 Hz

    def receive_pong(self):
        """Receive pong responses and calculate latency"""
        while self.running:
            try:
                data, addr = self.server_socket.recvfrom(1024)
                received_time = time.time()

                if data.decode('utf-8').startswith('PONG:'):
                    sent_timestamp = float(data.decode('utf-8')[5:])
                    latency = (received_time - sent_timestamp) * 1000  # ms
                    self.latency_samples.append(latency)
            except socket.timeout:
                continue

    def analyze_results(self):
        """Analyze latency test results"""
        if not self.latency_samples:
            return "No samples collected"

        return {
            'avg_latency': statistics.mean(self.latency_samples),
            'min_latency': min(self.latency_samples),
            'max_latency': max(self.latency_samples),
            'std_dev': statistics.stdev(self.latency_samples) if len(self.latency_samples) > 1 else 0,
            'sample_count': len(self.latency_samples),
            'percentile_95': sorted(self.latency_samples)[int(0.95 * len(self.latency_samples))]
        }
```

## Performance Optimization Checklist

### Before Implementation
- [ ] Profile current system to establish baseline
- [ ] Identify bottlenecks in communication pipeline
- [ ] Set realistic latency targets
- [ ] Plan monitoring and measurement approach

### During Implementation
- [ ] Use binary protocols instead of text-based
- [ ] Implement efficient data structures
- [ ] Use appropriate update frequencies
- [ ] Implement proper error handling
- [ ] Add performance monitoring

### After Implementation
- [ ] Measure end-to-end latency
- [ ] Monitor CPU and memory usage
- [ ] Test under various load conditions
- [ ] Validate synchronization accuracy
- [ ] Document performance characteristics

These examples demonstrate practical approaches to achieving sub-50ms synchronization between Gazebo and Unity, with emphasis on performance optimization and real-world implementation considerations.