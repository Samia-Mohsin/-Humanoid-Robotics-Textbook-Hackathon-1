---
sidebar_position: 20
---

# Kalman Filtering for Sensor Fusion

Kalman filtering is a fundamental technique in sensor fusion that provides optimal estimation of system states by combining noisy sensor measurements over time. This chapter explores the implementation of Kalman filters in digital twin environments for humanoid robotics, focusing on real-time state estimation and sensor data integration.

## Understanding Kalman Filters

The Kalman filter is a recursive algorithm that estimates the state of a dynamic system from a series of noisy measurements. It operates in two phases:

1. **Prediction**: Estimates the current state based on the previous state and system dynamics
2. **Update**: Refines the estimate using the latest sensor measurement

### Mathematical Foundation

The Kalman filter operates using the following state-space representation:

```
State Prediction: x̂ₖ|ₖ₋₁ = Fₖx̂ₖ₋₁|ₖ₋₁ + Bₖuₖ
Covariance Prediction: Pₖ|ₖ₋₁ = FₖPₖ₋₁|ₖ₋₁Fₖᵀ + Qₖ
Kalman Gain: Kₖ = Pₖ|ₖ₋₁Hₖᵀ(HₖPₖ|ₖ₋₁Hₖᵀ + Rₖ)⁻¹
State Update: x̂ₖ|ₖ = x̂ₖ|ₖ₋₁ + Kₖ(zₖ - Hₖx̂ₖ|ₖ₋₁)
Covariance Update: Pₖ|ₖ = (I - KₖHₖ)Pₖ|ₖ₋₁
```

Where:
- `x̂` is the state estimate vector
- `P` is the error covariance matrix
- `F` is the state transition model
- `H` is the observation model
- `Q` is the process noise covariance
- `R` is the observation noise covariance
- `z` is the measurement vector
- `K` is the Kalman gain

## Extended Kalman Filter (EKF) for Nonlinear Systems

For humanoid robots with nonlinear dynamics, the Extended Kalman Filter linearizes the system around the current estimate using Jacobian matrices:

```python
import numpy as np
from scipy.linalg import block_diag

class ExtendedKalmanFilter:
    def __init__(self, dim_x, dim_z, dt):
        """
        Extended Kalman Filter for nonlinear systems
        :param dim_x: State dimension
        :param dim_z: Measurement dimension
        :param dt: Time step
        """
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dt = dt

        # State vector [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.x = np.zeros(dim_x)

        # Covariance matrix
        self.P = np.eye(dim_x) * 1000.0

        # Process noise
        self.Q = np.eye(dim_x) * 0.1

        # Measurement noise
        self.R = np.eye(dim_z) * 1.0

        # Identity matrix
        self.I = np.eye(dim_x)

    def predict(self, F_func=None, Q=None):
        """
        Predict step for EKF
        :param F_func: Function that returns the Jacobian of the state transition
        :param Q: Process noise matrix (optional)
        """
        if F_func is not None:
            F = F_func(self.x, self.dt)
        else:
            # Default: constant velocity model
            F = self._default_jacobian()

        if Q is not None:
            self.Q = Q

        # State prediction
        self.x = self._state_transition(self.x, self.dt)

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z, H_func, R=None):
        """
        Update step for EKF
        :param z: Measurement vector
        :param H_func: Function that returns the Jacobian of the measurement function
        :param R: Measurement noise matrix (optional)
        """
        if R is not None:
            self.R = R

        H = H_func(self.x)

        # Innovation
        y = z - self._measurement_function(self.x)

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y

        # Covariance update
        self.P = (self.I - K @ H) @ self.P

    def _state_transition(self, x, dt):
        """Nonlinear state transition function for humanoid robot"""
        # Constant velocity model with rotation
        x_new = np.copy(x)

        # Update position based on velocity
        x_new[0] += x[3] * dt  # x position
        x_new[1] += x[4] * dt  # y position
        x_new[2] += x[5] * dt  # z position

        # Update velocities (with some decay)
        x_new[3] *= 0.99  # vx
        x_new[4] *= 0.99  # vy
        x_new[5] *= 0.99  # vz

        return x_new

    def _default_jacobian(self):
        """Default Jacobian for constant velocity model"""
        F = np.eye(self.dim_x)

        # Position updates based on velocity
        F[0, 3] = self.dt  # dx/dvx
        F[1, 4] = self.dt  # dy/dvy
        F[2, 5] = self.dt  # dz/dvz

        # Velocity decay
        F[3, 3] = 0.99
        F[4, 4] = 0.99
        F[5, 5] = 0.99

        return F

    def _measurement_function(self, x):
        """Nonlinear measurement function"""
        # For this example, we measure position directly
        # In practice, this would depend on sensor configuration
        return x[:self.dim_z]

# Example usage for humanoid robot state estimation
def humanoid_ekf_example():
    """Example of using EKF for humanoid robot state estimation"""

    # State: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    # Measurement: [x, y, z] from vision, [roll, pitch, yaw] from IMU
    ekf = ExtendedKalmanFilter(dim_x=9, dim_z=6, dt=0.01)  # 100Hz update rate

    # Initial state: robot at origin with zero velocity
    ekf.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Process noise (motion model uncertainty)
    ekf.Q = block_diag(
        np.eye(3) * 0.1,    # Position uncertainty
        np.eye(3) * 0.2,    # Velocity uncertainty
        np.eye(3) * 0.05    # Orientation uncertainty
    )

    # Measurement noise (sensor uncertainty)
    ekf.R = block_diag(
        np.eye(3) * 0.01,   # Vision position noise
        np.eye(3) * 0.001   # IMU orientation noise
    )

    # Measurement function Jacobian
    def H_func(x):
        # H maps full state to measurement space
        H = np.zeros((6, 9))
        H[0:3, 0:3] = np.eye(3)  # Position measurements
        H[3:6, 6:9] = np.eye(3)  # Orientation measurements
        return H

    # Simulate sensor measurements (in practice, these would come from actual sensors)
    for t in range(1000):  # Simulate 10 seconds at 100Hz
        # Simulate measurements with noise
        true_pos = np.array([0.1 * t * 0.01, 0.05 * np.sin(t * 0.01), 0.0])  # Simple trajectory
        true_orient = np.array([0.01 * np.sin(t * 0.01), 0.01 * np.cos(t * 0.01), 0.0])

        measurement = np.concatenate([
            true_pos + np.random.normal(0, 0.1, 3),      # Noisy position
            true_orient + np.random.normal(0, 0.01, 3)   # Noisy orientation
        ])

        # Prediction step
        ekf.predict()

        # Update step
        ekf.update(measurement, H_func)

        # The estimated state ekf.x now contains the fused estimate
        print(f"Time: {t*0.01:.2f}s, Estimated pos: [{ekf.x[0]:.3f}, {ekf.x[1]:.3f}, {ekf.x[2]:.3f}]")
```

## Unscented Kalman Filter (UKF) for Highly Nonlinear Systems

For systems with highly nonlinear dynamics, the Unscented Kalman Filter provides better accuracy by using deterministic sampling:

```python
class UnscentedKalmanFilter:
    def __init__(self, dim_x, dim_z, dt, alpha=1e-3, beta=2, kappa=0):
        """
        Unscented Kalman Filter implementation
        :param dim_x: State dimension
        :param dim_z: Measurement dimension
        :param dt: Time step
        :param alpha: Scaling parameter
        :param beta: Prior knowledge parameter
        :param kappa: Secondary scaling parameter
        """
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dt = dt

        # State vector
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x) * 1000.0

        # Process and measurement noise
        self.Q = np.eye(dim_x) * 0.1
        self.R = np.eye(dim_z) * 1.0

        # UKF parameters
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa

        # Calculate sigma point parameters
        lambda_ = alpha**2 * (dim_x + kappa) - dim_x
        self.gamma = np.sqrt(dim_x + lambda_)

        self.Wm = np.full(2 * dim_x + 1, 0.5 / (dim_x + lambda_))  # Mean weights
        self.Wm[0] = lambda_ / (dim_x + lambda_)

        self.Wc = np.full(2 * dim_x + 1, 0.5 / (dim_x + lambda_))  # Covariance weights
        self.Wc[0] = lambda_ / (dim_x + lambda_) + (1 - alpha**2 + beta)

    def predict(self, fx_func):
        """
        UKF prediction step
        :param fx_func: State transition function
        """
        # Calculate sigma points
        sigma_points = self._compute_sigma_points()

        # Propagate sigma points through state transition
        sigma_points_pred = np.zeros_like(sigma_points)
        for i in range(2 * self.dim_x + 1):
            sigma_points_pred[:, i] = fx_func(sigma_points[:, i], self.dt)

        # Calculate predicted mean and covariance
        self.x = np.sum(self.Wm[:, np.newaxis] * sigma_points_pred, axis=1)

        P_pred = np.zeros((self.dim_x, self.dim_x))
        for i in range(2 * self.dim_x + 1):
            diff = (sigma_points_pred[:, i] - self.x)
            P_pred += self.Wc[i] * np.outer(diff, diff)

        self.P = P_pred + self.Q

    def update(self, z, hx_func):
        """
        UKF update step
        :param z: Measurement vector
        :param hx_func: Measurement function
        """
        # Calculate sigma points
        sigma_points = self._compute_sigma_points()

        # Propagate sigma points through measurement function
        sigma_measurements = np.zeros((self.dim_z, 2 * self.dim_x + 1))
        for i in range(2 * self.dim_x + 1):
            sigma_measurements[:, i] = hx_func(sigma_points[:, i])

        # Calculate predicted measurement mean
        z_pred = np.sum(self.Wm[:, np.newaxis] * sigma_measurements, axis=1)

        # Calculate innovation covariance
        P_zz = np.zeros((self.dim_z, self.dim_z))
        for i in range(2 * self.dim_x + 1):
            diff = (sigma_measurements[:, i] - z_pred)
            P_zz += self.Wc[i] * np.outer(diff, diff)
        P_zz += self.R

        # Calculate cross-covariance
        P_xz = np.zeros((self.dim_x, self.dim_z))
        for i in range(2 * self.dim_x + 1):
            diff_x = (sigma_points[:, i] - self.x)
            diff_z = (sigma_measurements[:, i] - z_pred)
            P_xz += self.Wc[i] * np.outer(diff_x, diff_z)

        # Calculate Kalman gain
        K = P_xz @ np.linalg.inv(P_zz)

        # Update state and covariance
        self.x = self.x + K @ (z - z_pred)
        self.P = self.P - K @ P_zz @ K.T

    def _compute_sigma_points(self):
        """Calculate sigma points from current state and covariance"""
        U = np.linalg.cholesky(self.P + 1e-9 * np.eye(self.dim_x))

        sigma_points = np.zeros((self.dim_x, 2 * self.dim_x + 1))
        sigma_points[:, 0] = self.x

        for i in range(self.dim_x):
            sigma_points[:, i + 1] = self.x + self.gamma * U[:, i]
            sigma_points[:, i + 1 + self.dim_x] = self.x - self.gamma * U[:, i]

        return sigma_points

# Example: UKF for humanoid robot with complex dynamics
def humanoid_ukf_example():
    """Example of using UKF for humanoid robot with complex dynamics"""

    # State: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    ukf = UnscentedKalmanFilter(dim_x=9, dim_z=6, dt=0.01)

    # State transition function (nonlinear)
    def fx(state, dt):
        x, y, z, vx, vy, vz, roll, pitch, yaw = state

        # Apply nonlinear dynamics (e.g., humanoid with gravity, angular momentum)
        new_vx = vx + (-0.1 * vx) * dt  # Damping
        new_vy = vy + (-0.1 * vy) * dt
        new_vz = vz + (-9.81) * dt      # Gravity effect

        new_x = x + new_vx * dt
        new_y = y + new_vy * dt
        new_z = z + new_vz * dt

        # Simplified angular dynamics
        new_roll = roll + 0.1 * dt
        new_pitch = pitch + 0.05 * dt
        new_yaw = yaw + 0.2 * dt

        return np.array([new_x, new_y, new_z, new_vx, new_vy, new_vz, new_roll, new_pitch, new_yaw])

    # Measurement function
    def hx(state):
        # Return [x, y, z, roll, pitch, yaw] - what sensors measure
        return np.array([state[0], state[1], state[2], state[6], state[7], state[8]])

    # Simulate measurements and run UKF
    for t in range(1000):
        # Simulate measurement with noise
        measurement = np.array([
            0.1 * t * 0.01 + np.random.normal(0, 0.05),
            0.05 * np.sin(t * 0.01) + np.random.normal(0, 0.05),
            0.0 + np.random.normal(0, 0.02),
            0.01 * np.sin(t * 0.01) + np.random.normal(0, 0.005),
            0.01 * np.cos(t * 0.01) + np.random.normal(0, 0.005),
            0.0 + np.random.normal(0, 0.001)
        ])

        # Prediction step
        ukf.predict(fx)

        # Update step
        ukf.update(measurement, hx)

        print(f"UKF - Time: {t*0.01:.2f}s, State: [{ukf.x[0]:.3f}, {ukf.x[1]:.3f}, {ukf.x[2]:.3f}]")
```

## Multi-Sensor Kalman Filter Implementation

For digital twin environments, multiple sensors provide different types of measurements that need to be fused effectively:

```python
class MultiSensorKalmanFilter:
    def __init__(self, dim_state, dt):
        """
        Kalman filter that handles multiple sensor types
        :param dim_state: Dimension of the state vector
        :param dt: Time step
        """
        self.dim_state = dim_state
        self.dt = dt

        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, p_rate, y_rate, r_rate]
        self.x = np.zeros(dim_state)
        self.P = np.eye(dim_state) * 1000.0

        # Process noise
        self.Q = np.eye(dim_state) * 0.1

        # Initialize sensor-specific parameters
        self.sensors = {}
        self.active_measurements = {}

    def add_sensor(self, sensor_name, measurement_dim, observation_matrix, noise_covariance):
        """
        Add a sensor to the fusion system
        :param sensor_name: Name of the sensor
        :param measurement_dim: Dimension of the measurement vector
        :param observation_matrix: H matrix mapping state to measurement space
        :param noise_covariance: R matrix (measurement noise)
        """
        self.sensors[sensor_name] = {
            'dim': measurement_dim,
            'H': observation_matrix,
            'R': noise_covariance,
            'active': True
        }

    def predict(self):
        """Prediction step for the state"""
        # State transition matrix (constant velocity model with orientation)
        F = np.eye(self.dim_state)

        # Position updates based on velocity
        F[0, 3] = self.dt  # dx/dvx
        F[1, 4] = self.dt  # dy/dvy
        F[2, 5] = self.dt  # dz/dvz

        # Orientation updates based on angular rates
        F[6, 9] = self.dt  # droll/dp_rate
        F[7, 10] = self.dt # dpitch/dy_rate
        F[8, 11] = self.dt # dyaw/dr_rate

        # Apply prediction
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update_sensor(self, sensor_name, measurement):
        """
        Update filter with measurement from a specific sensor
        :param sensor_name: Name of the sensor
        :param measurement: Measurement vector
        """
        if sensor_name not in self.sensors or not self.sensors[sensor_name]['active']:
            return

        sensor = self.sensors[sensor_name]
        H = sensor['H']
        R = sensor['R']

        # Innovation
        y = measurement - H @ self.x

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y

        # Covariance update
        self.P = (np.eye(self.dim_state) - K @ H) @ self.P

        # Store this measurement for later reference
        self.active_measurements[sensor_name] = measurement

# Example: Multi-sensor fusion for humanoid robot
def multi_sensor_kalman_example():
    """Example of multi-sensor fusion using Kalman filter"""

    # State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, p_rate, y_rate, r_rate]
    kf = MultiSensorKalmanFilter(dim_state=12, dt=0.01)

    # LiDAR sensor - measures position [x, y, z]
    H_lidar = np.zeros((3, 12))
    H_lidar[0:3, 0:3] = np.eye(3)  # Maps position states to measurements
    R_lidar = np.eye(3) * 0.02     # Position uncertainty ~2cm
    kf.add_sensor('lidar', 3, H_lidar, R_lidar)

    # IMU sensor - measures orientation [roll, pitch, yaw] and angular rates
    H_imu = np.zeros((6, 12))
    H_imu[0:3, 6:9] = np.eye(3)    # Orientation
    H_imu[3:6, 9:12] = np.eye(3)   # Angular rates
    R_imu = np.diag([0.001, 0.001, 0.001, 0.01, 0.01, 0.01])  # Orientation: ~0.06 deg, Rates: ~0.6 deg/s
    kf.add_sensor('imu', 6, H_imu, R_imu)

    # Camera sensor - measures position in 2D [x, y] (with depth from other sensors)
    H_camera = np.zeros((2, 12))
    H_camera[0:2, 0:2] = np.eye(2)  # Maps x, y position to measurements
    R_camera = np.eye(2) * 0.05    # Position uncertainty ~5cm
    kf.add_sensor('camera', 2, H_camera, R_camera)

    # Simulate sensor measurements over time
    for t in range(1000):
        # Simulate measurements from different sensors with noise
        lidar_measurement = np.array([
            0.1 * t * 0.01 + np.random.normal(0, 0.02),  # x
            0.05 * np.sin(t * 0.01) + np.random.normal(0, 0.02),  # y
            0.0 + np.random.normal(0, 0.02)  # z
        ])

        imu_measurement = np.array([
            0.01 * np.sin(t * 0.01) + np.random.normal(0, 0.001),  # roll
            0.01 * np.cos(t * 0.01) + np.random.normal(0, 0.001),  # pitch
            0.0 + np.random.normal(0, 0.001),  # yaw
            0.01 * np.cos(t * 0.01) + np.random.normal(0, 0.01),   # p_rate
            -0.01 * np.sin(t * 0.01) + np.random.normal(0, 0.01),  # y_rate
            0.0 + np.random.normal(0, 0.01)   # r_rate
        ])

        camera_measurement = np.array([
            0.1 * t * 0.01 + np.random.normal(0, 0.05),  # x
            0.05 * np.sin(t * 0.01) + np.random.normal(0, 0.05)  # y
        ])

        # Prediction step
        kf.predict()

        # Update with each sensor measurement (in order of reliability)
        kf.update_sensor('lidar', lidar_measurement)
        kf.update_sensor('imu', imu_measurement)
        kf.update_sensor('camera', camera_measurement)

        # Print estimated state periodically
        if t % 100 == 0:
            print(f"Time: {t*0.01:.1f}s")
            print(f"Position: [{kf.x[0]:.3f}, {kf.x[1]:.3f}, {kf.x[2]:.3f}]")
            print(f"Orientation: [{kf.x[6]:.3f}, {kf.x[7]:.3f}, {kf.x[8]:.3f}]")
            print(f"Velocity: [{kf.x[3]:.3f}, {kf.x[4]:.3f}, {kf.x[5]:.3f}]")
            print("---")

# Run the example
if __name__ == "__main__":
    print("Extended Kalman Filter Example:")
    humanoid_ekf_example()
    print("\nUnscented Kalman Filter Example:")
    humanoid_ukf_example()
    print("\nMulti-Sensor Kalman Filter Example:")
    multi_sensor_kalman_example()
```

## Implementation in Digital Twin Environment

Integrating Kalman filtering into a digital twin requires careful consideration of the simulation environment and real-time constraints:

```python
# ROS/ROS2 node for Kalman filter in digital twin
import rospy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
from std_msgs.msg import Header
import message_filters

class DigitalTwinKalmanNode:
    def __init__(self):
        rospy.init_node('digital_twin_kalman_filter')

        # Initialize Kalman filter
        self.kf = MultiSensorKalmanFilter(dim_state=12, dt=0.01)

        # Set up subscribers for different sensors
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher for fused state estimate
        self.pose_pub = rospy.Publisher('/kalman_pose', PoseWithCovarianceStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/kalman_odom', Odometry, queue_size=10)

        # TF broadcaster for the robot's pose
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Timing for prediction updates
        self.prev_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(0.01), self.prediction_callback)  # 100Hz

        # Sensor synchronization
        self.latest_imu = None
        self.latest_odom = None

    def prediction_callback(self, event):
        """Prediction step at regular intervals"""
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        if dt > 0:
            # Update time step in the Kalman filter
            self.kf.dt = dt
            self.kf.predict()
            self.prev_time = current_time

            # Publish current estimate
            self.publish_estimate(current_time)

    def imu_callback(self, imu_msg):
        """Process IMU measurements"""
        # Extract orientation and angular velocities
        orientation = [imu_msg.orientation.x, imu_msg.orientation.y,
                      imu_msg.orientation.z, imu_msg.orientation.w]

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(orientation)

        angular_vel = [imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
                      imu_msg.angular_velocity.z]

        # Prepare measurement vector [roll, pitch, yaw, p_rate, y_rate, r_rate]
        measurement = np.array([roll, pitch, yaw] + angular_vel)

        # Update Kalman filter with IMU data
        self.kf.update_sensor('imu', measurement)

        # Store for synchronization
        self.latest_imu = (roll, pitch, yaw, angular_vel)

    def lidar_callback(self, lidar_msg):
        """Process LiDAR measurements (simplified - in practice, extract position from point cloud)"""
        # This is a simplified example; real implementation would process point cloud
        # to extract position/velocity estimates
        pass

    def odom_callback(self, odom_msg):
        """Process odometry measurements"""
        position = [odom_msg.pose.pose.position.x,
                   odom_msg.pose.pose.position.y,
                   odom_msg.pose.pose.position.z]

        linear_vel = [odom_msg.twist.twist.linear.x,
                     odom_msg.twist.twist.linear.y,
                     odom_msg.twist.twist.linear.z]

        # Prepare measurement vector [x, y, z, vx, vy, vz]
        measurement = np.array(position + linear_vel)

        # Update Kalman filter with odometry data
        self.kf.update_sensor('odometry', measurement)

        # Store for synchronization
        self.latest_odom = (position, linear_vel)

    def publish_estimate(self, timestamp):
        """Publish the current state estimate"""
        # Create PoseWithCovarianceStamped message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "map"

        # Set position from state estimate
        pose_msg.pose.pose.position.x = self.kf.x[0]
        pose_msg.pose.pose.position.y = self.kf.x[1]
        pose_msg.pose.pose.position.z = self.kf.x[2]

        # Convert orientation (simplified - would need proper conversion)
        pose_msg.pose.pose.orientation.w = 1.0  # Identity quaternion for now

        # Set covariance from the filter's covariance matrix
        pose_msg.pose.covariance = self.kf.P[:6, :6].flatten().tolist()

        # Publish the pose estimate
        self.pose_pub.publish(pose_msg)

        # Also publish as Odometry for compatibility
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = pose_msg.pose

        # Set twist (velocity) from state estimate
        odom_msg.twist.twist.linear.x = self.kf.x[3]
        odom_msg.twist.twist.linear.y = self.kf.x[4]
        odom_msg.twist.twist.linear.z = self.kf.x[5]

        # Set twist covariance
        twist_cov = np.zeros((6, 6))
        # Extract velocity covariance from state covariance
        twist_cov[0:3, 0:3] = self.kf.P[3:6, 3:6]
        twist_cov[3:6, 3:6] = self.kf.P[9:12, 9:12]  # Angular velocity covariance
        odom_msg.twist.covariance = twist_cov.flatten().tolist()

        self.odom_pub.publish(odom_msg)

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        import tf.transformations as tf_trans

        # Convert quaternion to rotation matrix, then to Euler angles
        euler = tf_trans.euler_from_quaternion(quat)
        return euler[0], euler[1], euler[2]  # roll, pitch, yaw

def main():
    """Main function to run the digital twin Kalman filter node"""
    node = DigitalTwinKalmanNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down Kalman filter node")

if __name__ == '__main__':
    main()
```

## Performance Optimization and Tuning

Kalman filters require careful tuning for optimal performance in digital twin environments:

### Parameter Tuning Guidelines

1. **Process Noise (Q)**: Higher values make the filter trust measurements more, lower values make it trust predictions more
2. **Measurement Noise (R)**: Should reflect actual sensor uncertainty characteristics
3. **Initial Covariance (P)**: Higher values mean less initial confidence in the state estimate

### Common Tuning Strategies

```python
def tune_kalman_parameters():
    """Guidelines for tuning Kalman filter parameters"""

    print("Kalman Filter Tuning Guidelines:")
    print("1. Start with high initial P to show uncertainty")
    print("2. Set R based on actual sensor specifications")
    print("3. Start with moderate Q and adjust based on:")
    print("   - Overshooting: Decrease Q")
    print("   - Lagging response: Increase Q")
    print("   - Noisy estimates: Decrease Q or increase R")

    # Example parameter sets for different scenarios
    scenarios = {
        "fast_moving_robot": {
            "Q_multiplier": 10.0,  # Higher process noise for dynamic movement
            "R_multiplier": 1.0    # Standard measurement noise
        },
        "stationary_robot": {
            "Q_multiplier": 0.1,   # Lower process noise for stable state
            "R_multiplier": 1.0    # Standard measurement noise
        },
        "high_precision_task": {
            "Q_multiplier": 1.0,   # Balanced process noise
            "R_multiplier": 0.1    # Trust measurements more
        }
    }

    return scenarios
```

## Best Practices for Digital Twin Implementation

1. **Modularity**: Design filters as reusable components
2. **Configurability**: Use parameter files for easy tuning
3. **Monitoring**: Track filter performance metrics
4. **Fail-Safe**: Implement fallback mechanisms when filters diverge
5. **Validation**: Continuously validate estimates against ground truth in simulation

Kalman filtering provides the mathematical foundation for effective sensor fusion in digital twin environments. By properly implementing and tuning these filters, you can achieve robust state estimation that enhances the realism and accuracy of your humanoid robot simulations.