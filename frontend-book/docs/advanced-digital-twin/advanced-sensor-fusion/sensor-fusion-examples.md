---
sidebar_position: 23
---

# Sensor Fusion Examples: Realistic Multi-Sensor Implementations

This chapter provides comprehensive, practical examples of sensor fusion implementations for humanoid robots in digital twin environments. These examples demonstrate real-world scenarios with realistic sensor configurations, noise models, and integration approaches.

## Complete Multi-Sensor Fusion System

Let's build a complete sensor fusion system that combines LiDAR, IMU, camera, and other sensors for a humanoid robot:

```python
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan, Imu, Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy.spatial.transform import Rotation as R
import cv2
from cv_bridge import CvBridge
import message_filters
from collections import deque
import threading
import time

class HumanoidMultiSensorFusion:
    def __init__(self):
        """
        Complete multi-sensor fusion system for humanoid robot
        Combines LiDAR, IMU, camera, and other sensors for state estimation
        """
        # Initialize node
        rospy.init_node('humanoid_sensor_fusion')

        # State vector: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
        self.state = np.zeros(12)
        self.covariance = np.eye(12) * 1000.0  # High initial uncertainty

        # Process noise
        self.Q = np.diag([
            0.1, 0.1, 0.2,      # Position noise
            0.01, 0.01, 0.02,   # Orientation noise
            0.5, 0.5, 0.5,      # Velocity noise
            0.1, 0.1, 0.1       # Angular velocity noise
        ])

        # Sensor data storage
        self.lidar_data = None
        self.imu_data = None
        self.camera_data = None
        self.odom_data = None

        # Timestamps for synchronization
        self.last_update_time = rospy.Time.now().to_sec()

        # Publishers
        self.pose_pub = rospy.Publisher('/fused_pose', PoseWithCovarianceStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/fused_odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/fused_imu', Imu, queue_size=10)

        # Subscribers
        self.setup_subscribers()

        # Sensor models
        self.lidar_model = self.setup_lidar_model()
        self.imu_model = self.setup_imu_model()
        self.camera_model = self.setup_camera_model()

        # Filtering
        self.ekf = self.initialize_ekf()

        # Threading for continuous processing
        self.processing_thread = threading.Thread(target=self.continuous_processing, daemon=True)
        self.processing_thread.start()

    def setup_subscribers(self):
        """Setup all sensor subscribers with approximate time synchronization"""
        # Use message_filters for time synchronization
        imu_sub = message_filters.Subscriber('/imu/data', Imu)
        lidar_sub = message_filters.Subscriber('/scan', LaserScan)

        # Synchronize messages with 0.1 second tolerance
        ts = message_filters.ApproximateTimeSynchronizer(
            [imu_sub, lidar_sub], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.sensor_sync_callback)

        # Additional individual subscribers
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        self.odom_sub = rospy.Subscriber('/wheel_odom', Odometry, self.odom_callback)

    def sensor_sync_callback(self, imu_msg, lidar_msg):
        """Callback for synchronized IMU and LiDAR data"""
        # Process IMU data
        self.imu_data = {
            'linear_acceleration': [imu_msg.linear_acceleration.x,
                                   imu_msg.linear_acceleration.y,
                                   imu_msg.linear_acceleration.z],
            'angular_velocity': [imu_msg.angular_velocity.x,
                                imu_msg.angular_velocity.y,
                                imu_msg.angular_velocity.z],
            'orientation': [imu_msg.orientation.x,
                           imu_msg.orientation.y,
                           imu_msg.orientation.z,
                           imu_msg.orientation.w],
            'timestamp': imu_msg.header.stamp.to_sec()
        }

        # Process LiDAR data
        ranges = np.array(lidar_msg.ranges)
        ranges = np.nan_to_num(ranges, nan=lidar_msg.range_max, posinf=lidar_msg.range_max, neginf=0.0)

        self.lidar_data = {
            'ranges': ranges,
            'intensities': np.array(lidar_msg.intensities),
            'angle_min': lidar_msg.angle_min,
            'angle_max': lidar_msg.angle_max,
            'angle_increment': lidar_msg.angle_increment,
            'time_increment': lidar_msg.time_increment,
            'scan_time': lidar_msg.scan_time,
            'range_min': lidar_msg.range_min,
            'range_max': lidar_msg.range_max,
            'timestamp': lidar_msg.header.stamp.to_sec()
        }

    def camera_callback(self, image_msg):
        """Process camera data"""
        try:
            # Convert ROS image to OpenCV
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Store for processing
            self.camera_data = {
                'image': cv_image,
                'encoding': image_msg.encoding,
                'height': image_msg.height,
                'width': image_msg.width,
                'timestamp': image_msg.header.stamp.to_sec()
            }
        except Exception as e:
            rospy.logwarn(f"Camera callback error: {e}")

    def odom_callback(self, odom_msg):
        """Process odometry data"""
        self.odom_data = {
            'position': [odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                        odom_msg.pose.pose.position.z],
            'orientation': [odom_msg.pose.pose.orientation.x,
                           odom_msg.pose.pose.orientation.y,
                           odom_msg.pose.pose.orientation.z,
                           odom_msg.pose.pose.orientation.w],
            'linear_velocity': [odom_msg.twist.twist.linear.x,
                               odom_msg.twist.twist.linear.y,
                               odom_msg.twist.twist.linear.z],
            'angular_velocity': [odom_msg.twist.twist.angular.x,
                                odom_msg.twist.twist.angular.y,
                                odom_msg.twist.twist.angular.z],
            'timestamp': odom_msg.header.stamp.to_sec(),
            'pose_covariance': np.array(odom_msg.pose.covariance).reshape(6, 6),
            'twist_covariance': np.array(odom_msg.twist.covariance).reshape(6, 6)
        }

    def setup_lidar_model(self):
        """Setup LiDAR sensor model"""
        return {
            'max_range': 30.0,
            'min_range': 0.1,
            'angular_resolution': 0.01745,  # 1 degree
            'range_accuracy': 0.02,  # 2cm
            'intensity_threshold': 10.0,
            'point_density': 360  # 360 points per revolution
        }

    def setup_imu_model(self):
        """Setup IMU sensor model"""
        return {
            'accel_noise_density': 0.017,  # mg/sqrt(Hz)
            'accel_bias_stability': 0.001,  # mg
            'gyro_noise_density': 0.001,   # deg/s/sqrt(Hz)
            'gyro_bias_stability': 0.0001, # deg/s
            'mag_noise_density': 0.1,      # uT/sqrt(Hz)
            'rate': 100  # Hz
        }

    def setup_camera_model(self):
        """Setup camera sensor model"""
        return {
            'fx': 525.0,  # Focal length x
            'fy': 525.0,  # Focal length y
            'cx': 319.5,  # Principal point x
            'cy': 239.5,  # Principal point y
            'k1': 0.0,    # Distortion coefficients
            'k2': 0.0,
            'p1': 0.0,
            'p2': 0.0,
            'k3': 0.0,
            'width': 640,
            'height': 480,
            'rate': 30  # Hz
        }

    def initialize_ekf(self):
        """Initialize Extended Kalman Filter parameters"""
        # State transition model (will be updated in predict step)
        self.F = np.eye(12)

        # Measurement models for different sensors
        self.H_lidar = np.zeros((2, 12))  # 2D position from LiDAR
        self.H_lidar[0, 0] = 1.0  # x measurement
        self.H_lidar[1, 1] = 1.0  # y measurement

        self.H_imu = np.zeros((6, 12))  # Orientation and angular velocity
        self.H_imu[0:4, 3:7] = np.eye(4)  # Orientation (quaternion)
        self.H_imu[4:6, 9:11] = np.eye(2)  # Angular velocity (wx, wy) - simplified

        self.H_odom = np.zeros((6, 12))  # Position and linear velocity
        self.H_odom[0:3, 0:3] = np.eye(3)  # Position
        self.H_odom[3:6, 6:9] = np.eye(3)  # Linear velocity

        return True  # Placeholder for EKF initialization

    def predict(self, dt):
        """Prediction step of the filter"""
        if dt <= 0:
            return

        # Update state transition matrix based on current state
        # For simplicity, using constant velocity model
        self.F = np.eye(12)

        # Position updates based on velocity
        self.F[0, 6] = dt  # x += vx * dt
        self.F[1, 7] = dt  # y += vy * dt
        self.F[2, 8] = dt  # z += vz * dt

        # Orientation updates based on angular velocity
        self.F[3, 9] = dt  # roll += wx * dt (simplified)
        self.F[4, 10] = dt # pitch += wy * dt
        self.F[5, 11] = dt # yaw += wz * dt

        # Update state prediction
        # This is a simplified prediction - in reality, you'd integrate the motion model
        self.state[0:3] += self.state[6:9] * dt  # Position from velocity
        self.state[3:6] += self.state[9:12] * dt  # Orientation from angular velocity

        # Update covariance prediction
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q * dt

    def update_lidar(self):
        """Update filter with LiDAR measurements"""
        if self.lidar_data is None:
            return

        # Extract 2D position from LiDAR data (simplified)
        # In practice, you'd perform more sophisticated processing
        ranges = self.lidar_data['ranges']
        angles = np.arange(
            self.lidar_data['angle_min'],
            self.lidar_data['angle_max'],
            self.lidar_data['angle_increment']
        )

        # Filter valid ranges
        valid_mask = (ranges >= self.lidar_data['range_min']) & (ranges <= self.lidar_data['range_max'])
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if len(valid_ranges) == 0:
            return

        # Calculate position estimate from LiDAR (simplified)
        # This would be much more sophisticated in a real implementation
        x_est = np.mean(valid_ranges * np.cos(valid_angles))
        y_est = np.mean(valid_ranges * np.sin(valid_angles))

        # Measurement vector [x, y]
        z = np.array([x_est, y_est])

        # Measurement noise
        R = np.diag([0.1, 0.1])  # 10cm standard deviation

        # Innovation
        y = z - (self.H_lidar @ self.state)[0:2]

        # Innovation covariance
        S = self.H_lidar[0:2, :] @ self.covariance @ self.H_lidar[0:2, :].T + R

        # Kalman gain
        K = self.covariance @ self.H_lidar[0:2, :].T @ np.linalg.inv(S)

        # State update
        self.state += K @ y

        # Covariance update
        self.covariance = (np.eye(12) - K @ self.H_lidar[0:2, :]) @ self.covariance

    def update_imu(self):
        """Update filter with IMU measurements"""
        if self.imu_data is None:
            return

        # Extract orientation from IMU
        quat_imu = np.array(self.imu_data['orientation'])

        # Extract angular velocity
        ang_vel_imu = np.array(self.imu_data['angular_velocity'])

        # Measurement vector [qx, qy, qz, qw, wx, wy] (simplified)
        z = np.concatenate([quat_imu, ang_vel_imu[0:2]])

        # Measurement noise
        R = np.diag([0.001, 0.001, 0.001, 0.001, 0.01, 0.01])  # Conservative estimates

        # Innovation
        y = z - (self.H_imu @ self.state)[0:6]

        # Innovation covariance
        S = self.H_imu @ self.covariance @ self.H_imu.T + R

        # Kalman gain
        K = self.covariance @ self.H_imu.T @ np.linalg.inv(S)

        # State update
        self.state += K @ y

        # Normalize quaternion part of state
        quat_norm = np.linalg.norm(self.state[3:7])
        if quat_norm > 0:
            self.state[3:7] /= quat_norm

        # Covariance update
        self.covariance = (np.eye(12) - K @ self.H_imu) @ self.covariance

    def update_odom(self):
        """Update filter with odometry measurements"""
        if self.odom_data is None:
            return

        # Extract position and velocity from odometry
        pos = np.array(self.odom_data['position'])[0:2]  # x, y only
        vel = np.array(self.odom_data['linear_velocity'])[0:2]  # vx, vy only

        # Measurement vector [x, y, vx, vy]
        z = np.concatenate([pos, vel])

        # Measurement noise from odometry covariance
        pos_cov = self.odom_data['pose_covariance'][0:2, 0:2]  # x, y covariance
        vel_cov = self.odom_data['twist_covariance'][0:2, 0:2]  # vx, vy covariance
        R = np.block([
            [pos_cov, np.zeros((2, 2))],
            [np.zeros((2, 2)), vel_cov]
        ])

        # Handle potential singular matrix
        R = R + np.eye(4) * 1e-6

        # Innovation
        y = z - (self.H_odom @ self.state)[0:4]

        # Innovation covariance
        S = self.H_odom[0:4, :] @ self.covariance @ self.H_odom[0:4, :].T + R

        # Kalman gain
        K = self.covariance @ self.H_odom[0:4, :].T @ np.linalg.inv(S)

        # State update
        self.state += K @ y

        # Covariance update
        self.covariance = (np.eye(12) - K @ self.H_odom[0:4, :]) @ self.covariance

    def continuous_processing(self):
        """Continuous processing loop"""
        rate = rospy.Rate(50)  # 50 Hz processing rate

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            dt = current_time - self.last_update_time

            if dt > 0:
                # Prediction step
                self.predict(dt)

                # Update steps with available sensor data
                self.update_lidar()
                self.update_imu()
                self.update_odom()

                # Publish results
                self.publish_results()

                self.last_update_time = current_time

            rate.sleep()

    def publish_results(self):
        """Publish fused state estimates"""
        current_time = rospy.Time.now()

        # Publish pose with covariance
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = self.state[0]
        pose_msg.pose.pose.position.y = self.state[1]
        pose_msg.pose.pose.position.z = self.state[2]

        # Use orientation from state (converted from roll, pitch, yaw to quaternion if needed)
        # For now, using identity quaternion as placeholder
        pose_msg.pose.pose.orientation.w = 1.0

        # Flatten covariance matrix
        pose_msg.pose.covariance = self.covariance[0:6, 0:6].flatten().tolist()
        self.pose_pub.publish(pose_msg)

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose = pose_msg.pose.pose
        odom_msg.pose.covariance = pose_msg.pose.covariance

        # Set twist (velocity)
        odom_msg.twist.twist.linear.x = self.state[6]
        odom_msg.twist.twist.linear.y = self.state[7]
        odom_msg.twist.twist.linear.z = self.state[8]
        odom_msg.twist.twist.angular.x = self.state[9]
        odom_msg.twist.twist.angular.y = self.state[10]
        odom_msg.twist.twist.angular.z = self.state[11]

        # Set twist covariance
        odom_msg.twist.covariance = self.covariance[6:12, 6:12].flatten().tolist()
        self.odom_pub.publish(odom_msg)

def main():
    """Main function to run the multi-sensor fusion node"""
    fusion_system = HumanoidMultiSensorFusion()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down multi-sensor fusion system")

if __name__ == '__main__':
    main()
```

## LiDAR-Inertial Odometry Integration

Here's an example of integrating LiDAR and IMU data for robust odometry:

```python
class LIOIntegration:
    def __init__(self):
        """
        LiDAR-Inertial Odometry Integration
        Combines LiDAR geometric information with IMU inertial measurements
        """
        # IMU bias states
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        # State: [position, velocity, orientation, accel_bias, gyro_bias]
        self.state = np.zeros(15)  # 3 pos + 3 vel + 4 quat + 3 accel_bias + 3 gyro_bias

        # Covariance matrix
        self.P = np.eye(15) * 0.1

        # Process noise
        self.Q = np.diag([
            0.1, 0.1, 0.2,      # Position process noise
            0.2, 0.2, 0.2,      # Velocity process noise
            0.01, 0.01, 0.01,   # Orientation process noise
            1e-6, 1e-6, 1e-6,   # Accel bias process noise
            1e-8, 1e-8, 1e-8    # Gyro bias process noise
        ])

        # Measurement noise
        self.R_lidar = np.diag([0.1, 0.1, 0.1])  # 10cm for each axis
        self.R_imu = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])  # Accel and gyro noise

        # Store IMU data for integration
        self.imu_buffer = deque(maxlen=100)
        self.lidar_poses = deque(maxlen=100)

    def predict_imu(self, dt, accel_measurement, gyro_measurement):
        """
        Predict state using IMU measurements
        :param dt: Time step
        :param accel_measurement: Raw accelerometer measurement
        :param gyro_measurement: Raw gyroscope measurement
        """
        # Correct measurements for bias
        accel_corrected = accel_measurement - self.accel_bias
        gyro_corrected = gyro_measurement - self.gyro_bias

        # Extract state components
        pos = self.state[0:3]
        vel = self.state[3:6]
        quat = self.state[6:10]  # quaternion: [w, x, y, z]
        accel_bias = self.state[10:13]
        gyro_bias = self.state[13:15]

        # Convert to scipy rotation for easier manipulation
        rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # [x, y, z, w] format

        # Acceleration in global frame
        gravity = np.array([0, 0, 9.81])
        accel_global = rot.apply(accel_corrected) + gravity

        # Update state
        new_pos = pos + vel * dt + 0.5 * accel_global * dt**2
        new_vel = vel + accel_global * dt

        # Update orientation using gyroscope
        angular_vel = gyro_corrected
        d_angle = np.linalg.norm(angular_vel) * dt
        if d_angle > 0:
            axis = angular_vel / d_angle
            dq = np.array([
                np.cos(d_angle/2),
                np.sin(d_angle/2) * axis[0],
                np.sin(d_angle/2) * axis[1],
                np.sin(d_angle/2) * axis[2]
            ])
            new_quat = self._quat_multiply(quat, dq)
        else:
            new_quat = quat

        # Bias random walk
        new_accel_bias = accel_bias
        new_gyro_bias = gyro_bias

        # Update state vector
        self.state = np.concatenate([
            new_pos, new_vel, new_quat, new_accel_bias, new_gyro_bias
        ])

        # Update covariance (simplified F matrix)
        F = self._compute_jacobian_imu(dt, accel_corrected, gyro_corrected)
        self.P = F @ self.P @ F.T + self.Q * dt

    def _compute_jacobian_imu(self, dt, accel, gyro):
        """Compute Jacobian matrix for IMU prediction"""
        F = np.eye(15)

        # Position-velocity relationship
        F[0:3, 3:6] = np.eye(3) * dt

        # Velocity-acceleration relationship
        # This is simplified - in reality, you'd need to account for rotation
        rot_matrix = R.from_quat([self.state[7], self.state[8], self.state[9], self.state[6]]).as_matrix()
        F[3:6, 6:10] = self._quat_to_rot_jacobian(accel, dt)  # Simplified

        return F

    def _quat_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    def _quat_to_rot_jacobian(self, accel, dt):
        """Jacobian of rotation matrix with respect to quaternion"""
        # Simplified implementation
        return np.zeros((3, 4))

    def update_lidar(self, lidar_position):
        """
        Update state with LiDAR-derived position
        :param lidar_position: Position estimate from LiDAR processing
        """
        # Measurement model: extract position from state
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)  # Position measurements

        # Innovation
        z_pred = H @ self.state  # Predicted position
        y = lidar_position - z_pred

        # Innovation covariance
        S = H @ self.P @ H.T + self.R_lidar

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.state += K @ y

        # Normalize quaternion part
        quat = self.state[6:10]
        quat_norm = np.linalg.norm(quat)
        if quat_norm > 0:
            self.state[6:10] = quat / quat_norm

        # Covariance update
        self.P = (np.eye(15) - K @ H) @ self.P

    def align_lidar_imu(self, lidar_poses, imu_data_list):
        """
        Align LiDAR and IMU data temporally and fuse consistently
        :param lidar_poses: List of LiDAR-derived poses
        :param imu_data_list: List of IMU measurements with timestamps
        """
        if len(lidar_poses) < 2 or len(imu_data_list) < 2:
            return

        # Time synchronization and alignment
        # This is a simplified approach - real implementation would be more sophisticated
        aligned_poses = []

        for lidar_pose in lidar_poses:
            # Find corresponding IMU data around the same time
            closest_imu = self._find_closest_imu(imu_data_list, lidar_pose['timestamp'])

            if closest_imu is not None:
                # Integrate IMU data to predict pose at LiDAR timestamp
                integrated_state = self._integrate_imu_to_time(
                    closest_imu, lidar_pose['timestamp']
                )

                # Update with LiDAR measurement
                self.update_lidar(lidar_pose['position'])

                aligned_poses.append({
                    'position': self.state[0:3],
                    'timestamp': lidar_pose['timestamp'],
                    'confidence': self._calculate_confidence()
                })

        return aligned_poses

    def _find_closest_imu(self, imu_data_list, target_time):
        """Find closest IMU measurement to target time"""
        if not imu_data_list:
            return None

        closest = min(imu_data_list, key=lambda x: abs(x['timestamp'] - target_time))
        return closest if abs(closest['timestamp'] - target_time) < 0.1 else None  # 100ms tolerance

    def _integrate_imu_to_time(self, start_imu, target_time):
        """Integrate IMU data from start time to target time"""
        # This would implement proper IMU integration
        # For this example, we'll return a placeholder
        return self.state

    def _calculate_confidence(self):
        """Calculate confidence based on covariance"""
        # Calculate trace of position covariance as uncertainty measure
        pos_cov = self.P[0:3, 0:3]
        uncertainty = np.trace(pos_cov)
        return 1.0 / (1.0 + uncertainty)  # Higher covariance = lower confidence

# Example usage of LIO integration
def lio_integration_example():
    """Example of LiDAR-IMU integration"""
    lio = LIOIntegration()

    # Simulate data processing
    for i in range(1000):
        # Simulate IMU data
        dt = 0.01  # 100Hz IMU
        accel = np.array([0.1 + 0.01 * np.random.randn(),
                         0.05 + 0.01 * np.random.randn(),
                         9.81 + 0.01 * np.random.randn()])  # With noise
        gyro = np.array([0.01 * np.random.randn(),
                        0.01 * np.random.randn(),
                        0.005 * np.random.randn()])  # Small angular rates

        # Predict with IMU
        lio.predict_imu(dt, accel, gyro)

        # Simulate LiDAR data every 10 IMU steps (10Hz LiDAR)
        if i % 10 == 0:
            lidar_pos = lio.state[0:3] + np.random.normal(0, 0.05, 3)  # Add LiDAR noise
            lio.update_lidar(lidar_pos)

        # Print status periodically
        if i % 100 == 0:
            print(f"Step {i}: Position = [{lio.state[0]:.3f}, {lio.state[1]:.3f}, {lio.state[2]:.3f}], "
                  f"Velocity = [{lio.state[3]:.3f}, {lio.state[4]:.3f}, {lio.state[5]:.3f}], "
                  f"Confidence = {lio._calculate_confidence():.3f}")
```

## Visual-Inertial Odometry (VIO) System

A comprehensive example of fusing camera and IMU data:

```python
class VIOSystem:
    def __init__(self):
        """
        Visual-Inertial Odometry System
        Combines visual features and IMU data for robust state estimation
        """
        # State vector: [position, velocity, orientation, gyro_bias, accel_bias]
        self.state = np.zeros(18)  # 3 pos + 3 vel + 4 quat + 3 gyro_bias + 3 accel_bias + 2 scale_factor
        self.P = np.eye(18) * 0.1  # Covariance

        # Process noise
        self.Q = np.diag([
            0.1, 0.1, 0.2,      # Position
            0.2, 0.2, 0.2,      # Velocity
            0.01, 0.01, 0.01,   # Orientation
            1e-6, 1e-6, 1e-6,   # Gyro bias
            1e-6, 1e-6, 1e-6,   # Accel bias
            0.01, 0.01,         # Scale factors
        ])

        # IMU parameters
        self.g = np.array([0, 0, 9.81])  # Gravity vector
        self.last_imu_time = None

        # Feature tracking
        self.feature_points = {}  # 3D feature positions in global frame
        self.feature_observations = []  # 2D image coordinates

    def initialize_with_features(self, image, features_2d, camera_params):
        """
        Initialize system with visual features
        :param image: Input image
        :param features_2d: 2D feature coordinates [u, v]
        :param camera_params: Camera intrinsic parameters
        """
        # Extract and match features (simplified)
        # In practice, you'd use ORB, SIFT, or other feature detectors

        # Initialize 3D points using stereo or motion (simplified depth assumption)
        for i, (u, v) in enumerate(features_2d):
            # Convert to normalized coordinates
            x_norm = (u - camera_params['cx']) / camera_params['fx']
            y_norm = (v - camera_params['cy']) / camera_params['fy']

            # Assume initial depth of 2 meters (would be computed from stereo or motion)
            z_world = 2.0
            x_world = x_norm * z_world
            y_world = y_norm * z_world

            self.feature_points[f'feature_{i}'] = np.array([x_world, y_world, z_world])

    def predict_with_imu(self, accel, gyro, dt):
        """
        Predict state using IMU measurements
        :param accel: Accelerometer measurement
        :param gyro: Gyroscope measurement
        :param dt: Time step
        """
        # Extract state components
        pos = self.state[0:3]
        vel = self.state[3:6]
        quat = self.state[6:10]
        gyro_bias = self.state[10:13]
        accel_bias = self.state[13:16]
        scale_factors = self.state[16:18]

        # Correct measurements
        accel_corrected = accel - accel_bias
        gyro_corrected = gyro - gyro_bias

        # Convert quaternion to rotation matrix
        rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()

        # Apply gravity and compute new acceleration
        accel_global = rotation @ accel_corrected + self.g

        # Update state
        new_pos = pos + vel * dt + 0.5 * accel_global * dt**2
        new_vel = vel + accel_global * dt

        # Update orientation
        angular_vel = gyro_corrected
        angle_magnitude = np.linalg.norm(angular_vel)
        if angle_magnitude > 1e-8:
            axis = angular_vel / angle_magnitude
            d_quat = R.from_rotvec(axis * angle_magnitude * dt).as_quat()
            d_quat = np.array([d_quat[3], d_quat[0], d_quat[1], d_quat[2]])  # w, x, y, z
            new_quat = self._quat_multiply(quat, d_quat)
        else:
            new_quat = quat

        # Normalize quaternion
        new_quat = new_quat / np.linalg.norm(new_quat)

        # Bias and scale factors (random walk model)
        new_gyro_bias = gyro_bias
        new_accel_bias = accel_bias
        new_scale_factors = scale_factors

        # Update state
        self.state = np.concatenate([
            new_pos, new_vel, new_quat,
            new_gyro_bias, new_accel_bias,
            new_scale_factors
        ])

        # Update covariance (F matrix computation simplified)
        self._update_covariance_prediction(dt, accel_corrected, gyro_corrected)

    def _update_covariance_prediction(self, dt, accel, gyro):
        """Update covariance using state transition model"""
        F = self._compute_state_transition_matrix(dt, accel, gyro)
        self.P = F @ self.P @ F.T + self.Q * dt

    def _compute_state_transition_matrix(self, dt, accel, gyro):
        """Compute state transition matrix F"""
        F = np.eye(18)

        # Position-velocity
        F[0:3, 3:6] = np.eye(3) * dt

        # Velocity-acceleration
        R = R.from_quat([self.state[7], self.state[8], self.state[9], self.state[6]]).as_matrix()
        F[3:6, 6:10] = self._rot_jacobian(R, accel) * dt  # Simplified

        # Orientation-angular velocity
        F[6:10, 10:13] = self._quat_omega_jacobian(gyro, dt)

        return F

    def _rot_jacobian(self, R, a):
        """Jacobian of rotation applied to acceleration"""
        # Simplified version
        return np.zeros((3, 4))

    def _quat_omega_jacobian(self, omega, dt):
        """Jacobian of quaternion update with angular velocity"""
        # Simplified implementation
        return np.zeros((4, 3))

    def update_with_features(self, features_2d, camera_params):
        """
        Update state using visual feature observations
        :param features_2d: Observed 2D feature positions
        :param camera_params: Camera intrinsic parameters
        """
        if len(features_2d) < 3:  # Need at least 3 features
            return

        # For each feature, compute expected measurement
        predicted_measurements = []
        observed_measurements = []
        measurement_indices = []

        for i, (u_obs, v_obs) in enumerate(features_2d):
            feature_key = f'feature_{i}'
            if feature_key in self.feature_points:
                # Transform 3D point to camera frame
                world_point = self.feature_points[feature_key]
                cam_point = self._transform_to_camera_frame(world_point)

                # Project to image coordinates
                u_pred = camera_params['fx'] * cam_point[0] / cam_point[2] + camera_params['cx']
                v_pred = camera_params['fy'] * cam_point[1] / cam_point[2] + camera_params['cy']

                predicted_measurements.extend([u_pred, v_pred])
                observed_measurements.extend([u_obs, v_obs])
                measurement_indices.extend([i*2, i*2+1])  # u, v indices

        if len(predicted_measurements) < 6:  # At least 3 features
            return

        # Measurement vector
        z_pred = np.array(predicted_measurements)
        z_obs = np.array(observed_measurements)

        # Measurement Jacobian (H matrix)
        H = self._compute_measurement_jacobian(measurement_indices, features_2d, camera_params)

        # Innovation
        y = z_obs - z_pred

        # Measurement noise
        R = np.eye(len(z_obs)) * 2.0  # 2 pixel uncertainty

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        self.state += K @ y

        # Normalize quaternion
        quat = self.state[6:10]
        self.state[6:10] = quat / np.linalg.norm(quat)

        # Covariance update
        self.P = (np.eye(18) - K @ H) @ self.P

    def _transform_to_camera_frame(self, world_point):
        """Transform point from world to camera frame"""
        # Extract pose from state
        pos = self.state[0:3]
        quat = self.state[6:10]

        # Convert quaternion to rotation matrix
        R_wc = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()

        # Transform: p_cam = R_cw * (p_world - t_cw)
        # We're assuming a simple camera mounted on the robot
        p_rel = world_point - pos
        p_cam = R_wc.T @ p_rel  # From world to camera frame

        return p_cam

    def _compute_measurement_jacobian(self, indices, features_2d, camera_params):
        """Compute measurement Jacobian H"""
        num_features = len(features_2d)
        H = np.zeros((num_features * 2, 18))

        for i, feature_idx in enumerate(indices):
            # Simplified Jacobian computation
            # In practice, this would involve complex derivatives of projection function
            pass

        return H

    def _quat_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

# Example usage of VIO system
def vio_example():
    """Example of Visual-Inertial Odometry"""
    vio = VIOSystem()

    # Simulate initialization with features
    dummy_features = [(100, 150), (200, 120), (300, 180), (400, 200)]  # 4 features
    camera_params = {'fx': 525, 'fy': 525, 'cx': 319.5, 'cy': 239.5}

    vio.initialize_with_features(None, dummy_features, camera_params)

    # Simulate motion with IMU and visual updates
    for i in range(1000):
        # Simulate IMU data
        dt = 0.01  # 100Hz
        accel = np.array([0.05 + 0.01 * np.random.randn(),
                         0.02 + 0.01 * np.random.randn(),
                         9.81 + 0.01 * np.random.randn()])
        gyro = np.array([0.005 * np.random.randn(),
                        0.003 * np.random.randn(),
                        0.001 * np.random.randn()])

        # Predict with IMU
        vio.predict_with_imu(accel, gyro, dt)

        # Update with visual features every 30 steps (3Hz visual updates)
        if i % 30 == 0:
            # Simulate feature tracking with some noise
            observed_features = [(u + np.random.normal(0, 0.5), v + np.random.normal(0, 0.5))
                               for u, v in dummy_features]
            vio.update_with_features(observed_features, camera_params)

        # Print status periodically
        if i % 100 == 0:
            pos = vio.state[0:3]
            vel = vio.state[3:6]
            print(f"VIO Step {i}: Position = [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], "
                  f"Velocity = [{vel[0]:.3f}, {vel[1]:.3f}, {vel[2]:.3f}]")
```

## Multi-Sensor SLAM Integration

Combining sensor fusion with mapping capabilities:

```python
class MultiSensorSLAM:
    def __init__(self):
        """
        Multi-Sensor SLAM: Simultaneous Localization and Mapping
        Integrates LiDAR, camera, and IMU for robust mapping and localization
        """
        # Robot state: position, orientation, velocity
        self.robot_state = np.zeros(9)  # x, y, z, roll, pitch, yaw, vx, vy, vz
        self.state_covariance = np.eye(9) * 0.1

        # Map representation (simplified grid map)
        self.grid_map = np.zeros((200, 200))  # 200x200 grid
        self.map_resolution = 0.1  # 10cm per cell
        self.map_origin = np.array([-10.0, -10.0])  # Map origin in world coordinates

        # Feature map (landmarks)
        self.landmarks = {}  # Dictionary of landmarks with IDs
        self.landmark_covariances = {}  # Covariance for each landmark

        # Sensor fusion weights
        self.sensor_weights = {
            'lidar': 1.0,
            'camera': 0.8,
            'imu': 0.6,
            'odom': 0.4
        }

        # Motion model noise
        self.motion_noise = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.05, 0.05, 0.05])

    def update_with_lidar(self, point_cloud, robot_pose):
        """
        Update SLAM with LiDAR data
        :param point_cloud: LiDAR point cloud data
        :param robot_pose: Robot's current pose estimate
        """
        # Convert point cloud to occupancy grid
        grid_updates = self._points_to_grid(point_cloud, robot_pose)

        # Update grid map with new measurements
        self._update_grid_map(grid_updates)

        # Extract features/landmarks from point cloud
        features = self._extract_features_from_points(point_cloud)

        # Add new landmarks or update existing ones
        for feature_id, feature_pos in features.items():
            world_pos = self._transform_to_world(feature_pos, robot_pose)
            self._add_or_update_landmark(feature_id, world_pos)

    def update_with_camera(self, image_features, camera_pose):
        """
        Update SLAM with visual features
        :param image_features: Detected features in image coordinates
        :param camera_pose: Camera's current pose
        """
        # Convert image features to 3D points (with depth or triangulation)
        for feature_2d in image_features:
            # In practice, you'd use stereo or depth to get 3D positions
            # Here we'll simulate the conversion
            landmark_3d = self._feature_to_3d(feature_2d, camera_pose)
            if landmark_3d is not None:
                self._add_or_update_landmark(f"vis_{hash(str(feature_2d))}", landmark_3d)

    def update_with_imu(self, imu_data, dt):
        """
        Update robot state using IMU data
        :param imu_data: IMU measurements
        :param dt: Time step
        """
        # Extract measurements
        linear_acc = np.array(imu_data['linear_acceleration'])
        angular_vel = np.array(imu_data['angular_velocity'])

        # Update state prediction
        new_state = self.robot_state.copy()

        # Integrate acceleration to get velocity and position
        rot_matrix = self._euler_to_rotation_matrix(
            self.robot_state[3:6]  # roll, pitch, yaw
        )

        # Transform acceleration to global frame
        global_acc = rot_matrix @ linear_acc + np.array([0, 0, 9.81])

        # Update state
        new_state[0:3] += self.robot_state[6:9] * dt + 0.5 * global_acc * dt**2  # Position
        new_state[6:9] += global_acc * dt  # Velocity
        new_state[3:6] += angular_vel * dt  # Orientation (simplified)

        # Update state and covariance
        self.robot_state = new_state
        self.state_covariance += self.motion_noise * dt

    def _points_to_grid(self, point_cloud, robot_pose):
        """Convert LiDAR points to grid map updates"""
        updates = []

        for point in point_cloud:
            # Transform point to world coordinates
            world_point = self._transform_to_world(point, robot_pose)

            # Convert to grid coordinates
            grid_x = int((world_point[0] - self.map_origin[0]) / self.map_resolution)
            grid_y = int((world_point[1] - self.map_origin[1]) / self.map_resolution)

            if (0 <= grid_x < self.grid_map.shape[0] and
                0 <= grid_y < self.grid_map.shape[1]):
                updates.append((grid_x, grid_y))

        return updates

    def _transform_to_world(self, local_point, robot_pose):
        """Transform local point to world coordinates"""
        x_r, y_r, z_r = robot_pose[:3]
        roll, pitch, yaw = robot_pose[3:6]

        # Create rotation matrix
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        rotation_matrix = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw, cos_yaw, 0],
            [0, 0, 1]
        ])

        # Rotate and translate
        world_point = rotation_matrix @ local_point[:3] + np.array([x_r, y_r, z_r])
        return np.append(world_point, local_point[3] if len(local_point) > 3 else 0)

    def _update_grid_map(self, occupied_cells):
        """Update occupancy grid with new measurements"""
        # Simple occupancy update - set occupied cells to 0.8
        for x, y in occupied_cells:
            if 0 <= x < self.grid_map.shape[0] and 0 <= y < self.grid_map.shape[1]:
                self.grid_map[x, y] = max(self.grid_map[x, y], 0.8)

    def _extract_features_from_points(self, point_cloud):
        """Extract distinctive features from point cloud"""
        features = {}

        # Simple approach: cluster points and find centers
        if len(point_cloud) > 10:
            # This is a simplified feature extraction
            # In practice, you'd use more sophisticated methods like NDT, ICP, etc.
            point_clusters = self._cluster_points(point_cloud)

            for i, cluster in enumerate(point_clusters):
                if len(cluster) > 5:  # Only large clusters
                    center = np.mean(cluster, axis=0)
                    features[f'lidar_{i}'] = center

        return features

    def _cluster_points(self, points):
        """Simple clustering of 3D points"""
        # Simplified clustering using spatial distance
        clusters = []
        used = set()

        for i, point in enumerate(points):
            if i in used:
                continue

            cluster = [point]
            used.add(i)

            # Find nearby points
            for j, other_point in enumerate(points):
                if j in used:
                    continue

                if np.linalg.norm(point - other_point) < 0.5:  # 50cm threshold
                    cluster.append(other_point)
                    used.add(j)

            if len(cluster) > 1:  # Only keep clusters with multiple points
                clusters.append(cluster)

        return clusters

    def _add_or_update_landmark(self, landmark_id, position):
        """Add new landmark or update existing landmark position"""
        if landmark_id not in self.landmarks:
            # New landmark
            self.landmarks[landmark_id] = position.copy()
            self.landmark_covariances[landmark_id] = np.eye(3) * 0.1
        else:
            # Update existing landmark using EKF update
            self._update_landmark_position(landmark_id, position)

    def _update_landmark_position(self, landmark_id, new_observation):
        """Update landmark position using EKF update"""
        old_pos = self.landmarks[landmark_id]
        old_cov = self.landmark_covariances[landmark_id]

        # Measurement model: directly observe landmark position
        H = np.eye(3)  # Direct observation matrix

        # Innovation
        innovation = new_observation - old_pos

        # Innovation covariance
        R = np.eye(3) * 0.04  # Observation noise (20cm std)
        S = H @ old_cov @ H.T + R

        # Kalman gain
        K = old_cov @ H.T @ np.linalg.inv(S)

        # Update landmark position and covariance
        self.landmarks[landmark_id] = old_pos + K @ innovation
        self.landmark_covariances[landmark_id] = (np.eye(3) - K @ H) @ old_cov

    def _feature_to_3d(self, feature_2d, camera_pose):
        """Convert 2D image feature to 3D landmark (simplified)"""
        # In a real system, you'd use triangulation from multiple views or depth
        # Here we'll just place the feature at a fixed distance ahead of the camera
        u, v = feature_2d
        fx, fy = 525.0, 525.0  # Camera focal lengths
        cx, cy = 319.5, 239.5  # Principal point

        # Convert to normalized coordinates
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy

        # Assume fixed depth of 3 meters
        depth = 3.0
        x_cam = x_norm * depth
        y_cam = y_norm * depth
        z_cam = depth

        # Transform from camera to world frame
        camera_pos = camera_pose[:3]
        camera_rot = self._euler_to_rotation_matrix(camera_pose[3:6])

        point_cam = np.array([x_cam, y_cam, z_cam])
        point_world = camera_rot @ point_cam + camera_pos

        return point_world

    def _euler_to_rotation_matrix(self, euler_angles):
        """Convert Euler angles to rotation matrix"""
        roll, pitch, yaw = euler_angles

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        rotation_matrix = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])

        return rotation_matrix

    def get_robot_pose(self):
        """Get current robot pose"""
        return self.robot_state[:6]  # x, y, z, roll, pitch, yaw

    def get_map(self):
        """Get current map"""
        return self.grid_map.copy()

    def get_landmarks(self):
        """Get current landmarks"""
        return self.landmarks.copy()

# Example usage of Multi-Sensor SLAM
def slam_example():
    """Example of Multi-Sensor SLAM operation"""
    slam = MultiSensorSLAM()

    # Simulate robot moving and collecting sensor data
    for step in range(500):
        # Simulate robot movement in a square pattern
        time_step = step * 0.1  # 10Hz simulation
        robot_x = 2.0 * np.cos(time_step * 0.5)  # Circular motion
        robot_y = 2.0 * np.sin(time_step * 0.5)
        robot_z = 0.0

        robot_pose = np.array([
            robot_x, robot_y, robot_z,  # Position
            0.0, 0.0, time_step * 0.5,  # Orientation (yaw changing)
            0.0, 0.0, 0.0              # Velocity
        ])

        # Simulate LiDAR data (points in a circle around robot)
        lidar_points = []
        for angle in np.linspace(0, 2*np.pi, 360):
            distance = 3.0 + 0.5 * np.sin(5 * angle)  # Create an interesting shape
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            z = 0.0
            lidar_points.append([x, y, z])

        # Update SLAM with LiDAR data
        slam.update_with_lidar(np.array(lidar_points), robot_pose)

        # Simulate IMU data
        imu_dt = 0.01  # 100Hz IMU
        imu_data = {
            'linear_acceleration': [0.1 * np.cos(time_step), 0.1 * np.sin(time_step), 9.81],
            'angular_velocity': [0.0, 0.0, 0.5]  # Constant rotation
        }
        slam.update_with_imu(imu_data, imu_dt)

        # Simulate camera features (simplified)
        camera_features = [(100 + i*50, 150) for i in range(5)]  # Simulate 5 features
        slam.update_with_camera(camera_features, robot_pose)

        # Print status periodically
        if step % 100 == 0:
            pose = slam.get_robot_pose()
            print(f"SLAM Step {step}: Robot at [{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}], "
                  f"Yaw: {pose[5]:.2f}, Landmarks: {len(slam.get_landmarks())}")

    # Print final statistics
    final_pose = slam.get_robot_pose()
    landmarks = slam.get_landmarks()
    print(f"\nFinal SLAM status:")
    print(f"Robot position: [{final_pose[0]:.2f}, {final_pose[1]:.2f}, {final_pose[2]:.2f}]")
    print(f"Total landmarks mapped: {len(landmarks)}")
    print(f"Map shape: {slam.get_map().shape}")
```

## Sensor Fusion for Humanoid Balance Control

An example of using sensor fusion specifically for humanoid robot balance control:

```python
class HumanoidBalanceController:
    def __init__(self):
        """
        Humanoid Balance Controller using sensor fusion
        Combines IMU, force/torque sensors, and kinematic data for balance control
        """
        # State representation for balance
        self.com_state = np.zeros(6)  # [x, y, z, vx, vy, vz] - Center of Mass
        self.com_covariance = np.eye(6) * 0.1

        # Zero Moment Point (ZMP) state
        self.zmp = np.zeros(2)  # [x, y] - Zero Moment Point

        # Support polygon (foot positions)
        self.support_polygon = np.array([
            [-0.1, -0.05], [0.1, -0.05], [0.1, 0.05], [-0.1, 0.05]  # Simplified rectangular foot
        ])

        # Sensor data buffers
        self.imu_buffer = deque(maxlen=10)
        self.foot_force_buffer = deque(maxlen=10)
        self.joint_angle_buffer = deque(maxlen=10)

        # Control parameters
        self.zmp_ref = np.array([0.0, 0.0])  # Desired ZMP
        self.com_ref = np.array([0.0, 0.0, 0.8])  # Desired CoM height

        # Process noise
        self.Q = np.diag([0.01, 0.01, 0.02, 0.05, 0.05, 0.05])

        # Sensor noise
        self.R_imu = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
        self.R_force = np.diag([5.0, 5.0, 5.0])  # 5N force uncertainty

    def update_balance_state(self, imu_data, force_torque_data, joint_angles):
        """
        Update balance state using multiple sensors
        :param imu_data: IMU measurements [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
        :param force_torque_data: Force/torque sensor data [f_x, f_y, f_z, t_x, t_y, t_z]
        :param joint_angles: Joint angle measurements
        """
        # Store sensor data
        self.imu_buffer.append(imu_data)
        self.foot_force_buffer.append(force_torque_data)
        self.joint_angle_buffer.append(joint_angles)

        # Predict CoM state using IMU
        self._predict_com_state(imu_data)

        # Update using force/torque measurements
        self._update_with_force_torque(force_torque_data)

        # Compute ZMP from force measurements
        self._compute_zmp_from_forces(force_torque_data)

    def _predict_com_state(self, imu_data):
        """Predict CoM state using IMU measurements"""
        acc = np.array(imu_data[:3])  # Linear acceleration
        dt = 0.01  # Assuming 100Hz update

        # Update state: position += velocity * dt + 0.5 * acceleration * dt^2
        self.com_state[0:3] += self.com_state[3:6] * dt + 0.5 * acc * dt**2
        # Update velocity: velocity += acceleration * dt
        self.com_state[3:6] += acc * dt

        # Update covariance
        A = np.eye(6)
        A[0:3, 3:6] = np.eye(3) * dt  # Position-velocity relationship
        self.com_covariance = A @ self.com_covariance @ A.T + self.Q * dt

    def _update_with_force_torque(self, force_torque_data):
        """Update CoM estimate using force/torque measurements"""
        # Extract forces
        forces = np.array(force_torque_data[:3])

        # Expected force based on CoM acceleration and gravity
        expected_force = 70 * (self.com_state[3:6] / 0.01) + np.array([0, 0, 70 * 9.81])  # 70kg robot

        # Innovation
        y = forces - expected_force

        # Measurement matrix (simplified)
        H = np.block([np.zeros((3, 3)), 70/0.01 * np.eye(3)])  # Mapping velocity to force

        # Innovation covariance
        S = H @ self.com_covariance @ H.T + self.R_force

        # Kalman gain
        K = self.com_covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.com_state += K @ y

        # Update covariance
        self.com_covariance = (np.eye(6) - K @ H) @ self.com_covariance

    def _compute_zmp_from_forces(self, force_torque_data):
        """Compute Zero Moment Point from force/torque measurements"""
        f_x, f_y, f_z, t_x, t_y, t_z = force_torque_data

        # ZMP calculation: zmp_x = x_cop - t_y / f_z, zmp_y = y_cop + t_x / f_z
        # where cop is center of pressure
        cop_x, cop_y = 0.0, 0.0  # Assume center of foot for now

        if abs(f_z) > 1e-6:  # Avoid division by zero
            self.zmp[0] = cop_x - t_y / f_z
            self.zmp[1] = cop_y + t_x / f_z
        else:
            self.zmp[0] = cop_x
            self.zmp[1] = cop_y

    def is_balanced(self):
        """Check if the robot is balanced based on ZMP"""
        # Check if ZMP is within support polygon
        return self._point_in_polygon(self.zmp, self.support_polygon)

    def _point_in_polygon(self, point, polygon):
        """Check if point is inside polygon using ray casting algorithm"""
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def compute_balance_control(self):
        """
        Compute balance control commands based on sensor fusion results
        """
        # Compute error between current and desired ZMP
        zmp_error = self.zmp_ref - self.zmp

        # Compute CoM error
        com_error = self.com_ref - np.append(self.com_state[0:2], self.com_ref[2])  # Keep reference height

        # Simple PD controller for balance
        kp_zmp = 10.0
        kd_zmp = 2.0
        kp_com = 5.0
        kd_com = 1.0

        control_output = kp_zmp * zmp_error + kd_zmp * (zmp_error - self._get_prev_zmp_error())
        control_output += kp_com * com_error[0:2] + kd_com * (com_error[0:2] - self._get_prev_com_error())

        return control_output

    def _get_prev_zmp_error(self):
        """Get previous ZMP error (simplified)"""
        return np.zeros(2)

    def _get_prev_com_error(self):
        """Get previous CoM error (simplified)"""
        return np.zeros(2)

# Example usage of Humanoid Balance Controller
def balance_control_example():
    """Example of humanoid balance control"""
    controller = HumanoidBalanceController()

    # Simulate balance control over time
    for step in range(1000):
        # Simulate sensor data
        imu_data = [0.01 * np.random.randn(), 0.01 * np.random.randn(), 9.81 + 0.1 * np.random.randn(),
                   0.001 * np.random.randn(), 0.001 * np.random.randn(), 0.0005 * np.random.randn()]

        force_data = [10 * np.random.randn(), 10 * np.random.randn(), 686.7 + 50 * np.random.randn(),  # ~70kg * 9.81
                     0.1 * np.random.randn(), 0.1 * np.random.randn(), 0.05 * np.random.randn()]

        joint_angles = [0.1 * np.random.randn() for _ in range(12)]  # 12 joint angles

        # Update balance state
        controller.update_balance_state(imu_data, force_data, joint_angles)

        # Check balance
        balanced = controller.is_balanced()

        # Compute control commands
        control_cmd = controller.compute_balance_control()

        # Print status periodically
        if step % 200 == 0:
            print(f"Balance Step {step}: ZMP = [{controller.zmp[0]:.3f}, {controller.zmp[1]:.3f}], "
                  f"CoM = [{controller.com_state[0]:.3f}, {controller.com_state[1]:.3f}], "
                  f"Balanced = {balanced}")

if __name__ == "__main__":
    # Run all examples
    print("Running multi-sensor fusion system example...")
    fusion_system = HumanoidMultiSensorFusion()
    print("Multi-sensor fusion system initialized.")

    print("\nRunning LiDAR-IMU integration example...")
    lio_integration_example()

    print("\nRunning Visual-Inertial Odometry example...")
    vio_example()

    print("\nRunning Multi-Sensor SLAM example...")
    slam_example()

    print("\nRunning Humanoid Balance Control example...")
    balance_control_example()

    print("\nAll sensor fusion examples completed!")
```

## Performance Optimization and Best Practices

Here are some important considerations for implementing efficient sensor fusion systems:

### 1. Efficient Data Processing
- Use efficient data structures for sensor data storage
- Implement data buffering and batch processing where possible
- Consider sensor data rate matching and interpolation

### 2. Computational Efficiency
- Optimize matrix operations using libraries like NumPy
- Consider using sparse matrices when appropriate
- Implement efficient algorithms for large-scale problems

### 3. Robustness Considerations
- Handle sensor failures gracefully
- Implement outlier rejection techniques
- Monitor filter consistency and reset when needed

### 4. Calibration and Validation
- Regularly calibrate sensors for optimal performance
- Validate fusion results against ground truth when available
- Monitor system performance and adjust parameters accordingly

These realistic multi-sensor fusion implementations provide practical examples for developing robust sensor fusion systems in digital twin environments for humanoid robots. Each example demonstrates different aspects of sensor fusion and can be adapted for specific applications and requirements.