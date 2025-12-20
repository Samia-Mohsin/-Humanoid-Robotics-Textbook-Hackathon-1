---
sidebar_position: 12
---

# Network Communication Protocols

Effective network communication is fundamental to maintaining synchronization between Gazebo physics and Unity visualization. This chapter covers various protocols and approaches for transmitting data between platforms with minimal latency.

## Communication Architecture

The communication between Gazebo and Unity typically follows a client-server or publisher-subscriber model:

- **Gazebo as Publisher**: Gazebo publishes physics state data
- **Unity as Subscriber**: Unity subscribes to receive state updates
- **Bidirectional Communication**: For cases where Unity needs to send control commands back to Gazebo

## Protocol Options

### 1. ROS/ROS2 Communication

The most common approach uses ROS (Robot Operating System) to bridge Gazebo and Unity:

```python
# Gazebo ROS Publisher Example
import rospy
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64

class GazeboStatePublisher:
    def __init__(self):
        rospy.init_node('gazebo_state_publisher')

        # Publisher for link states
        self.link_states_pub = rospy.Publisher(
            '/gazebo/link_states',
            LinkStates,
            queue_size=10
        )

        # Publisher for custom messages
        self.custom_pub = rospy.Publisher(
            '/synchronization_data',
            CustomSyncMsg,
            queue_size=10
        )

        self.rate = rospy.Rate(100)  # 100 Hz update rate

    def publish_states(self):
        while not rospy.is_shutdown():
            # Get current link states from Gazebo
            link_states = self.get_gazebo_link_states()

            # Publish to Unity subscriber
            self.link_states_pub.publish(link_states)

            # Publish additional synchronization data
            sync_msg = self.create_sync_message()
            self.custom_pub.publish(sync_msg)

            self.rate.sleep()
```

```csharp
// Unity ROS Subscriber Example
using UnityEngine;
using RosSharp;

public class UnityStateSubscriber : MonoBehaviour
{
    private RosSocket rosSocket;
    private string gazeboTopic = "/gazebo/link_states";

    void Start()
    {
        // Connect to ROS bridge
        rosSocket = new RosSocket("ws://localhost:9090");

        // Subscribe to Gazebo link states
        rosSocket.Subscribe<LinkStates>(gazeboTopic, OnLinkStatesReceived);
    }

    private void OnLinkStatesReceived(LinkStates linkStates)
    {
        // Update Unity objects based on received states
        for (int i = 0; i < linkStates.name.Count; i++)
        {
            string linkName = linkStates.name[i];
            var pose = linkStates.pose[i];

            UpdateUnityObject(linkName, pose);
        }
    }

    private void UpdateUnityObject(string name, Pose pose)
    {
        GameObject obj = GameObject.Find(name);
        if (obj != null)
        {
            obj.transform.position = ConvertPosition(pose.position);
            obj.transform.rotation = ConvertRotation(pose.orientation);
        }
    }

    private Vector3 ConvertPosition(geometry_msgs.Point pos)
    {
        // Convert from Gazebo to Unity coordinate system
        return new Vector3((float)pos.y, (float)pos.z, (float)pos.x);
    }

    private Quaternion ConvertRotation(geometry_msgs.Quaternion quat)
    {
        return new Quaternion(
            (float)quat.y,
            (float)quat.z,
            (float)quat.x,
            -(float)quat.w
        );
    }
}
```

### 2. TCP/IP Socket Communication

For direct communication without ROS dependencies:

```python
# Gazebo TCP Server
import socket
import json
import threading
import time

class GazeboTCPServer:
    def __init__(self, host='localhost', port=8080):
        self.host = host
        self.port = port
        self.clients = []
        self.running = True

    def start_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)

        print(f"Server listening on {self.host}:{self.port}")

        # Accept connections in a separate thread
        accept_thread = threading.Thread(target=self.accept_connections)
        accept_thread.start()

        # Send data updates in main thread
        self.send_updates()

    def accept_connections(self):
        while self.running:
            try:
                client_socket, address = self.server_socket.accept()
                print(f"Connection from {address}")
                self.clients.append(client_socket)

                # Start client handler thread
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket,)
                )
                client_thread.start()
            except:
                break

    def send_updates(self):
        while self.running:
            # Get current simulation state
            state_data = self.get_simulation_state()

            # Send to all connected clients
            for client in self.clients[:]:  # Copy list to avoid modification during iteration
                try:
                    json_data = json.dumps(state_data)
                    client.send(json_data.encode('utf-8') + b'\n')
                except:
                    # Remove disconnected client
                    if client in self.clients:
                        self.clients.remove(client)

            time.sleep(0.01)  # 100Hz update rate

    def get_simulation_state(self):
        # This would interface with Gazebo to get current state
        return {
            "timestamp": time.time(),
            "objects": [
                {
                    "name": "robot_link_1",
                    "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            ]
        }
```

```csharp
// Unity TCP Client
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using System.Collections;

public class UnityTCPClient : MonoBehaviour
{
    private TcpClient tcpClient;
    private NetworkStream stream;
    private Thread receiveThread;
    private bool running = false;

    [SerializeField] private string host = "localhost";
    [SerializeField] private int port = 8080;

    void Start()
    {
        ConnectToServer();
    }

    private void ConnectToServer()
    {
        try
        {
            tcpClient = new TcpClient(host, port);
            stream = tcpClient.GetStream();
            running = true;

            // Start receiving data in a separate thread
            receiveThread = new Thread(ReceiveData);
            receiveThread.Start();
        }
        catch (System.Exception e)
        {
            Debug.LogError("Connection failed: " + e.Message);
        }
    }

    private void ReceiveData()
    {
        byte[] buffer = new byte[1024];

        while (running)
        {
            try
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    string data = System.Text.Encoding.UTF8.GetString(buffer, 0, bytesRead);

                    // Process received data on main thread
                    UnityEngine.WSA.Application.InvokeOnAppThread(() => {
                        ProcessReceivedData(data);
                    }, false);
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError("Receive error: " + e.Message);
                break;
            }
        }
    }

    private void ProcessReceivedData(string jsonData)
    {
        // Parse JSON and update Unity objects
        // Implementation depends on your data structure
    }

    void OnApplicationQuit()
    {
        running = false;
        if (stream != null) stream.Close();
        if (tcpClient != null) tcpClient.Close();
        if (receiveThread != null) receiveThread.Join();
    }
}
```

### 3. UDP Communication

For low-latency scenarios where occasional packet loss is acceptable:

```python
# Gazebo UDP Broadcaster
import socket
import json
import time

class GazeboUDPBroadcaster:
    def __init__(self, host='255.255.255.255', port=8888):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.host = host
        self.port = port

    def broadcast_state(self):
        while True:
            state_data = self.get_simulation_state()
            json_data = json.dumps(state_data)

            self.sock.sendto(json_data.encode('utf-8'), (self.host, self.port))
            time.sleep(0.01)  # 100Hz update rate

    def get_simulation_state(self):
        # Get current simulation state from Gazebo
        return {
            "timestamp": time.time(),
            "objects": [
                {
                    "name": "robot_link_1",
                    "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            ]
        }
```

```csharp
// Unity UDP Listener
using System.Net;
using System.Net.Sockets;
using UnityEngine;
using System.Threading;

public class UnityUDPListener : MonoBehaviour
{
    private UdpClient udpClient;
    private Thread receiveThread;
    private bool running = false;

    [SerializeField] private int port = 8888;

    void Start()
    {
        StartListener();
    }

    private void StartListener()
    {
        udpClient = new UdpClient(port);
        running = true;

        receiveThread = new Thread(ReceiveData);
        receiveThread.Start();
    }

    private void ReceiveData()
    {
        IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, port);

        while (running)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remoteEP);
                string jsonData = System.Text.Encoding.UTF8.GetString(data);

                // Process on main thread
                UnityEngine.WSA.Application.InvokeOnAppThread(() => {
                    ProcessReceivedData(jsonData);
                }, false);
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
        // Parse and update Unity objects
    }

    void OnApplicationQuit()
    {
        running = false;
        if (udpClient != null) udpClient.Close();
        if (receiveThread != null) receiveThread.Join();
    }
}
```

## Protocol Selection Guidelines

### Use ROS/ROS2 when:
- You already have a ROS-based robotics infrastructure
- You need rich message types and services
- You want to leverage existing ROS tools and packages
- You're working in a robotics research environment

### Use TCP/IP when:
- You need guaranteed message delivery
- You want to avoid ROS dependencies
- You require custom message formats
- You need reliable communication for critical applications

### Use UDP when:
- You prioritize low latency over reliability
- Occasional packet loss is acceptable
- You're broadcasting to multiple clients
- You have bandwidth constraints

## Performance Considerations

### Message Frequency
- Balance update rate with network capacity
- Consider interpolation on the receiving end for lower update rates
- Typical rates: 50-200 Hz for smooth visualization

### Message Size
- Minimize data transmission by sending only necessary information
- Consider delta updates instead of full state
- Compress data if bandwidth is limited

### Network Optimization
- Use local networks when possible
- Consider multicast for multiple Unity instances
- Implement data buffering and interpolation

## Troubleshooting Common Issues

### High Latency
1. Check network bandwidth and quality
2. Reduce message size or frequency
3. Use faster serialization formats
4. Optimize both Gazebo and Unity performance

### Packet Loss
1. Switch from UDP to TCP if reliability is critical
2. Check network infrastructure
3. Reduce transmission frequency
4. Implement retransmission mechanisms

### Desynchronization
1. Implement time synchronization
2. Use interpolation/extrapolation
3. Monitor and log timing information
4. Consider clock drift compensation

These protocols provide the foundation for reliable communication between Gazebo and Unity, enabling the real-time synchronization necessary for effective digital twin implementations.