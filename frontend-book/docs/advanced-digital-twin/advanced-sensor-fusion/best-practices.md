---
sidebar_position: 24
---

# Best Practices for Sensor Fusion Implementation

This chapter outlines essential best practices for implementing robust, efficient, and reliable sensor fusion systems in digital twin environments for humanoid robotics. These practices are derived from real-world implementations and research in multi-sensor integration.

## Design Principles

### 1. Modular Architecture
Design your sensor fusion system with modular components that can be independently developed, tested, and maintained:

```python
class ModularSensorFusion:
    """
    Example of modular sensor fusion architecture
    """
    def __init__(self):
        # Sensor modules
        self.imu_module = IMUSensorModule()
        self.lidar_module = LIDARSensorModule()
        self.camera_module = CameraSensorModule()

        # Fusion engine
        self.fusion_engine = FusionEngine()

        # State estimator
        self.state_estimator = StateEstimator()

        # Validation module
        self.validator = ValidationModule()

    def process_sensor_data(self, sensor_data):
        """
        Process data through modular pipeline
        """
        # Preprocess sensor data
        processed_data = {}
        for sensor_type, data in sensor_data.items():
            if sensor_type == 'imu':
                processed_data[sensor_type] = self.imu_module.process(data)
            elif sensor_type == 'lidar':
                processed_data[sensor_type] = self.lidar_module.process(data)
            elif sensor_type == 'camera':
                processed_data[sensor_type] = self.camera_module.process(data)

        # Fuse processed data
        fused_data = self.fusion_engine.fuse(processed_data)

        # Update state estimate
        state_estimate = self.state_estimator.update(fused_data)

        # Validate results
        validation_result = self.validator.validate(state_estimate)

        return state_estimate, validation_result

class IMUSensorModule:
    """
    IMU-specific processing module
    """
    def __init__(self):
        self.bias_estimator = BiasEstimator()
        self.orientation_filter = OrientationFilter()

    def process(self, raw_data):
        # Bias correction
        corrected_data = self.bias_estimator.correct(raw_data)

        # Orientation estimation
        orientation = self.orientation_filter.estimate(corrected_data)

        return {
            'orientation': orientation,
            'linear_acceleration': corrected_data['acceleration'],
            'angular_velocity': corrected_data['gyro']
        }

class LIDARSensorModule:
    """
    LiDAR-specific processing module
    """
    def __init__(self):
        self.outlier_filter = OutlierFilter()
        self.feature_extractor = FeatureExtractor()

    def process(self, raw_data):
        # Filter outliers
        filtered_data = self.outlier_filter.remove_outliers(raw_data)

        # Extract features
        features = self.feature_extractor.extract(filtered_data)

        return {
            'features': features,
            'point_cloud': filtered_data,
            'intensity_data': raw_data.get('intensities', [])
        }

class CameraSensorModule:
    """
    Camera-specific processing module
    """
    def __init__(self):
        self.image_processor = ImageProcessor()
        self.feature_detector = FeatureDetector()

    def process(self, raw_data):
        # Process image
        processed_image = self.image_processor.process(raw_data)

        # Detect features
        features = self.feature_detector.detect(processed_image)

        return {
            'features': features,
            'processed_image': processed_image,
            'camera_pose': raw_data.get('camera_pose', None)
        }

class FusionEngine:
    """
    Generic fusion engine that can work with different sensor types
    """
    def __init__(self):
        self.fusion_methods = {
            'kalman': KalmanFusion(),
            'particle': ParticleFusion(),
            'bayesian': BayesianFusion()
        }
        self.current_method = 'kalman'

    def fuse(self, sensor_data):
        return self.fusion_methods[self.current_method].fuse(sensor_data)

class StateEstimator:
    """
    State estimation module
    """
    def __init__(self):
        self.state_vector = np.zeros(12)  # [pos, vel, orient, ang_vel]
        self.covariance = np.eye(12)

    def update(self, fused_data):
        # Update state based on fused data
        # Implementation depends on fusion method used
        pass

class ValidationModule:
    """
    Validation and consistency checking
    """
    def __init__(self):
        self.consistency_checkers = [
            InnovationConsistencyChecker(),
            CovarianceConsistencyChecker(),
            PhysicalConsistencyChecker()
        ]

    def validate(self, state_estimate):
        results = {}
        for checker in self.consistency_checkers:
            results[checker.name] = checker.check(state_estimate)
        return results
```

### 2. Data-Driven Design
Base your fusion algorithms on the characteristics of your specific sensors and application requirements:

- **Characterize sensor noise**: Measure and model the actual noise characteristics of your sensors
- **Understand sensor limitations**: Know the operational limits, accuracy, and failure modes
- **Consider environmental factors**: Account for how environmental conditions affect sensor performance

### 3. Scalable Architecture
Design systems that can accommodate additional sensors without major architectural changes:

```python
class ScalableFusionSystem:
    """
    Scalable sensor fusion system that can accommodate new sensors
    """
    def __init__(self):
        self.sensors = {}
        self.fusion_strategies = {}
        self.sensor_weights = {}

    def register_sensor(self, sensor_name, sensor_processor, fusion_strategy):
        """
        Register a new sensor with its processing module and fusion strategy
        """
        self.sensors[sensor_name] = sensor_processor
        self.fusion_strategies[sensor_name] = fusion_strategy
        self.sensor_weights[sensor_name] = 1.0  # Default weight

    def add_sensor_weight_callback(self, sensor_name, callback_func):
        """
        Add a callback to dynamically adjust sensor weights based on conditions
        """
        self.sensor_weights[sensor_name] = callback_func

    def process_fusion(self, sensor_data):
        """
        Process fusion with dynamic sensor weighting
        """
        weighted_measurements = {}

        for sensor_name, data in sensor_data.items():
            if sensor_name in self.sensors:
                # Process sensor data
                processed_data = self.sensors[sensor_name].process(data)

                # Get dynamic weight
                if callable(self.sensor_weights[sensor_name]):
                    weight = self.sensor_weights[sensor_name](processed_data)
                else:
                    weight = self.sensor_weights[sensor_name]

                # Apply weight to measurements
                weighted_measurements[sensor_name] = {
                    'data': processed_data,
                    'weight': weight,
                    'covariance': processed_data.get('covariance', np.eye(3)) / weight
                }

        # Perform weighted fusion
        return self._perform_weighted_fusion(weighted_measurements)

    def _perform_weighted_fusion(self, weighted_measurements):
        """
        Perform fusion using weighted measurements
        """
        # Implementation depends on specific fusion algorithm
        # Could use weighted least squares, covariance intersection, etc.
        pass
```

## Implementation Best Practices

### 1. Efficient Data Structures
Use appropriate data structures for optimal performance:

```python
from collections import deque
import numpy as np

class EfficientDataStructures:
    """
    Best practices for efficient data structures in sensor fusion
    """
    def __init__(self, buffer_size=1000):
        # Use deques for time-series data with fixed size
        self.imu_buffer = deque(maxlen=buffer_size)
        self.lidar_buffer = deque(maxlen=buffer_size)
        self.camera_buffer = deque(maxlen=buffer_size)

        # Use numpy arrays for numerical computations
        self.state_vector = np.zeros(12, dtype=np.float64)
        self.covariance_matrix = np.eye(12, dtype=np.float64)

        # Use dictionaries for sensor-specific parameters
        self.sensor_params = {}

    def add_imu_data(self, timestamp, data):
        """
        Add IMU data efficiently
        """
        self.imu_buffer.append({
            'timestamp': timestamp,
            'data': np.array(data, dtype=np.float64),
            'processed': False
        })

    def get_recent_data(self, sensor_type, n_recent=10):
        """
        Get recent data efficiently
        """
        if sensor_type == 'imu':
            return list(self.imu_buffer)[-n_recent:]
        elif sensor_type == 'lidar':
            return list(self.lidar_buffer)[-n_recent:]
        elif sensor_type == 'camera':
            return list(self.camera_buffer)[-n_recent:]
        return []
```

### 2. Memory Management
Optimize memory usage for real-time systems:

```python
class MemoryEfficientFusion:
    """
    Memory-efficient sensor fusion implementation
    """
    def __init__(self):
        # Pre-allocate arrays to avoid memory allocation during runtime
        self.working_matrices = {
            'kalman_gain': np.zeros((12, 6)),
            'innovation': np.zeros(6),
            'temp_state': np.zeros(12),
            'temp_cov': np.zeros((12, 12))
        }

        # Use memory views when possible
        self.state_memory_view = None
        self.covariance_memory_view = None

    def predict_step(self, dt):
        """
        Memory-efficient prediction step
        """
        # Use pre-allocated arrays to avoid creating new ones
        F = self._compute_state_transition_matrix(dt)

        # Update state in place
        np.dot(F, self.state_vector, out=self.working_matrices['temp_state'])
        self.state_vector[:] = self.working_matrices['temp_state']

        # Update covariance in place
        temp = F @ self.covariance_matrix
        np.dot(temp, F.T, out=self.working_matrices['temp_cov'])
        self.covariance_matrix[:] = self.working_matrices['temp_cov']
        self.covariance_matrix += self.process_noise * dt

    def update_step(self, measurement, measurement_model, measurement_noise):
        """
        Memory-efficient update step
        """
        # Calculate innovation
        expected_measurement = measurement_model @ self.state_vector
        innovation = measurement - expected_measurement

        # Calculate innovation covariance
        temp = measurement_model @ self.covariance_matrix
        innovation_cov = temp @ measurement_model.T + measurement_noise

        # Calculate Kalman gain
        kalman_gain = self.covariance_matrix @ measurement_model.T
        kalman_gain = kalman_gain @ np.linalg.inv(innovation_cov)

        # Update state
        self.state_vector += kalman_gain @ innovation

        # Update covariance
        I_KH = np.eye(len(self.state_vector)) - kalman_gain @ measurement_model
        self.covariance_matrix = I_KH @ self.covariance_matrix
```

### 3. Numerical Stability
Ensure numerical stability in your algorithms:

```python
class NumericallyStableFusion:
    """
    Numerically stable sensor fusion implementation
    """
    def __init__(self):
        self.state_vector = np.zeros(12)
        self.covariance_matrix = np.eye(12)
        self.cholesky_factor = np.eye(12)  # For numerical stability

    def update_with_cholesky(self, measurement, H, R):
        """
        Update using Cholesky decomposition for numerical stability
        """
        # Calculate innovation
        innovation = measurement - H @ self.state_vector

        # Calculate innovation covariance using Cholesky
        S = H @ self.covariance_matrix @ H.T + R

        # Use Cholesky decomposition for numerical stability
        try:
            L = np.linalg.cholesky(S)
            # Solve using forward and backward substitution
            y = np.linalg.solve(L, innovation)
            K = np.linalg.solve(L.T, y)
        except np.linalg.LinAlgError:
            # Fallback to standard inversion if Cholesky fails
            K = np.linalg.solve(S, innovation)

        # Update state
        self.state_vector += self.covariance_matrix @ H.T @ K

        # Update covariance using Joseph form for stability
        I_KH = np.eye(len(self.state_vector)) - np.outer(K, H)
        self.covariance_matrix = I_KH @ self.covariance_matrix @ I_KH.T + \
                                np.outer(K, K) * R

    def enforce_symmetry(self):
        """
        Enforce symmetry of covariance matrix
        """
        self.covariance_matrix = (self.covariance_matrix + self.covariance_matrix.T) / 2.0

    def ensure_positive_definite(self, min_eigenvalue=1e-9):
        """
        Ensure covariance matrix is positive definite
        """
        eigenvals, eigenvecs = np.linalg.eigh(self.covariance_matrix)

        # Ensure all eigenvalues are positive
        eigenvals = np.maximum(eigenvals, min_eigenvalue)

        # Reconstruct matrix
        self.covariance_matrix = eigenvecs @ np.diag(eigenvals) @ eigenvecs.T
```

## Robustness and Fault Tolerance

### 1. Sensor Failure Handling
Implement graceful degradation when sensors fail:

```python
class RobustFusionSystem:
    """
    Robust sensor fusion with failure handling
    """
    def __init__(self):
        self.sensors = {}
        self.sensor_health = {}
        self.fallback_strategies = {}
        self.last_valid_measurements = {}

    def register_sensor(self, name, sensor_obj, fallback_strategy=None):
        """
        Register a sensor with optional fallback strategy
        """
        self.sensors[name] = sensor_obj
        self.sensor_health[name] = {'healthy': True, 'last_update': 0}
        self.fallback_strategies[name] = fallback_strategy
        self.last_valid_measurements[name] = None

    def process_with_health_check(self, current_time):
        """
        Process sensor data with health checking
        """
        valid_data = {}

        for sensor_name, sensor_obj in self.sensors.items():
            try:
                # Check sensor health
                measurement = sensor_obj.get_measurement()

                # Validate measurement
                if self._validate_measurement(sensor_name, measurement):
                    valid_data[sensor_name] = measurement
                    self.sensor_health[sensor_name]['healthy'] = True
                    self.sensor_health[sensor_name]['last_update'] = current_time
                    self.last_valid_measurements[sensor_name] = measurement
                else:
                    # Measurement validation failed
                    self._handle_invalid_measurement(sensor_name, measurement, current_time)
            except Exception as e:
                # Sensor failed completely
                self._handle_sensor_failure(sensor_name, e, current_time)

        return self._fuse_valid_data(valid_data, current_time)

    def _validate_measurement(self, sensor_name, measurement):
        """
        Validate sensor measurement
        """
        # Check for NaN or inf values
        if np.any(np.isnan(measurement)) or np.any(np.isinf(measurement)):
            return False

        # Check for reasonable ranges
        if sensor_name == 'imu':
            # Check acceleration magnitude (shouldn't exceed ~10g normally)
            accel_magnitude = np.linalg.norm(measurement[:3])
            if accel_magnitude > 100:  # 10g threshold
                return False

        elif sensor_name == 'lidar':
            # Check for reasonable range values
            ranges = measurement if isinstance(measurement, np.ndarray) else np.array(measurement)
            if np.any(ranges < 0.05) or np.any(ranges > 100.0):  # Unreasonable ranges
                return False

        return True

    def _handle_invalid_measurement(self, sensor_name, measurement, current_time):
        """
        Handle invalid sensor measurement
        """
        self.sensor_health[sensor_name]['healthy'] = False

        # Switch to fallback if available
        fallback = self.fallback_strategies[sensor_name]
        if fallback:
            self._apply_fallback(sensor_name, fallback)

    def _handle_sensor_failure(self, sensor_name, error, current_time):
        """
        Handle complete sensor failure
        """
        self.sensor_health[sensor_name]['healthy'] = False
        print(f"Sensor {sensor_name} failed: {error}")

        # Apply fallback strategy
        fallback = self.fallback_strategies[sensor_name]
        if fallback:
            self._apply_fallback(sensor_name, fallback)

    def _apply_fallback(self, sensor_name, fallback_strategy):
        """
        Apply fallback strategy for failed sensor
        """
        # Implementation depends on specific fallback requirements
        # Could use prediction, interpolation, or other sensor data
        pass

    def _fuse_valid_data(self, valid_data, current_time):
        """
        Fuse only valid sensor data
        """
        # Adjust fusion weights based on sensor health
        weights = {}
        for sensor_name in valid_data.keys():
            health_score = self._get_health_score(sensor_name, current_time)
            weights[sensor_name] = health_score

        # Perform weighted fusion
        return self._weighted_fusion(valid_data, weights)

    def _get_health_score(self, sensor_name, current_time):
        """
        Calculate health score for sensor (0.0 to 1.0)
        """
        health_info = self.sensor_health[sensor_name]

        if not health_info['healthy']:
            return 0.0

        # Calculate time-based degradation
        time_since_update = current_time - health_info['last_update']
        if time_since_update > 1.0:  # 1 second threshold
            return 0.0

        # Additional health checks could be implemented here
        return 1.0
```

### 2. Outlier Rejection
Implement robust outlier detection and rejection:

```python
class OutlierRejectionFusion:
    """
    Sensor fusion with robust outlier rejection
    """
    def __init__(self):
        self.innovation_threshold = 3.0  # 3-sigma threshold
        self.robust_weights = {}
        self.innovation_history = deque(maxlen=100)

    def update_with_outlier_rejection(self, measurement, H, R, sensor_name="unknown"):
        """
        Update with outlier rejection using innovation-based detection
        """
        # Calculate innovation
        expected_measurement = H @ self.state_vector
        innovation = measurement - expected_measurement

        # Calculate innovation covariance
        S = H @ self.covariance_matrix @ H.T + R

        # Calculate normalized innovation squared (NIS)
        try:
            S_inv = np.linalg.inv(S)
            nis = innovation.T @ S_inv @ innovation

            # Store for history-based outlier detection
            self.innovation_history.append(nis)

            # Calculate robust weight based on innovation
            weight = self._calculate_robust_weight(nis, len(measurement))

            if weight < 0.1:  # Consider as outlier if weight is too low
                print(f"Outlier detected from {sensor_name}, innovation: {nis:.3f}")
                return False  # Indicate outlier was detected

            # Apply robust update
            self._robust_update(measurement, H, R, weight)
            return True

        except np.linalg.LinAlgError:
            # Matrix is singular, skip update
            return False

    def _calculate_robust_weight(self, nis, dof):
        """
        Calculate robust weight using various methods
        """
        # Method 1: Huber weight
        huber_threshold = chi2.ppf(0.95, dof)  # 95% confidence
        if nis <= huber_threshold:
            weight = 1.0
        else:
            weight = huber_threshold / nis  # Decrease weight for large innovations

        return weight

    def _robust_update(self, measurement, H, R, weight):
        """
        Perform robust update with weighted measurements
        """
        # Weighted measurement and noise
        weighted_measurement = weight * measurement
        weighted_R = R / weight

        # Standard Kalman update with weighted values
        innovation = weighted_measurement - H @ self.state_vector
        S = H @ self.covariance_matrix @ H.T + weighted_R

        try:
            K = self.covariance_matrix @ H.T @ np.linalg.inv(S)
            self.state_vector += K @ innovation

            I_KH = np.eye(len(self.state_vector)) - K @ H
            self.covariance_matrix = I_KH @ self.covariance_matrix @ I_KH.T + \
                                    K @ weighted_R @ K.T
        except np.linalg.LinAlgError:
            # If matrix inversion fails, skip update
            pass

    def mahalanobis_outlier_detection(self, measurement, expected_measurement, covariance):
        """
        Detect outliers using Mahalanobis distance
        """
        diff = measurement - expected_measurement
        try:
            inv_cov = np.linalg.inv(covariance)
            distance_squared = diff.T @ inv_cov @ diff
            distance = np.sqrt(distance_squared)

            # Compare with threshold (typically 2-3 for 95-99% confidence)
            threshold = 3.0
            return distance > threshold
        except np.linalg.LinAlgError:
            # If covariance is singular, use simpler check
            std_dev = np.sqrt(np.diag(covariance))
            normalized_diff = np.abs(diff) / std_dev
            return np.any(normalized_diff > 3.0)
```

## Performance Optimization

### 1. Real-time Considerations
Optimize for real-time performance:

```python
import time
from threading import Thread, Lock
import queue

class RealTimeFusionSystem:
    """
    Real-time optimized sensor fusion system
    """
    def __init__(self, target_frequency=100):  # 100 Hz target
        self.target_period = 1.0 / target_frequency
        self.processing_times = deque(maxlen=100)
        self.fusion_lock = Lock()

        # Queues for sensor data
        self.imu_queue = queue.Queue(maxsize=10)
        self.lidar_queue = queue.Queue(maxsize=5)  # Lower frequency
        self.camera_queue = queue.Queue(maxsize=5)  # Lower frequency

        # Threading for parallel processing
        self.processing_thread = Thread(target=self._processing_loop, daemon=True)
        self.processing_thread.start()

    def _processing_loop(self):
        """
        Main processing loop for real-time fusion
        """
        while True:
            start_time = time.time()

            # Process sensor data
            self._process_available_data()

            # Perform fusion
            self._perform_fusion()

            # Calculate processing time
            processing_time = time.time() - start_time
            self.processing_times.append(processing_time)

            # Sleep to maintain target frequency
            sleep_time = max(0, self.target_period - processing_time)
            time.sleep(sleep_time)

    def _process_available_data(self):
        """
        Process all available sensor data
        """
        # Process IMU data (high frequency)
        while not self.imu_queue.empty():
            try:
                imu_data = self.imu_queue.get_nowait()
                self._process_imu_data(imu_data)
            except queue.Empty:
                break

        # Process LiDAR data (lower frequency)
        try:
            lidar_data = self.lidar_queue.get_nowait()
            self._process_lidar_data(lidar_data)
        except queue.Empty:
            pass  # Not an error, just no data available

        # Process camera data (lower frequency)
        try:
            camera_data = self.camera_queue.get_nowait()
            self._process_camera_data(camera_data)
        except queue.Empty:
            pass  # Not an error, just no data available

    def _perform_fusion(self):
        """
        Perform sensor fusion with time constraints
        """
        with self.fusion_lock:
            # Use the most recent data for fusion
            # Prioritize high-frequency sensors
            self._integrate_imu_data()
            self._integrate_lidar_data()
            self._integrate_camera_data()

    def get_average_processing_time(self):
        """
        Get average processing time for performance monitoring
        """
        if self.processing_times:
            return sum(self.processing_times) / len(self.processing_times)
        return 0.0

    def get_utilization_percentage(self):
        """
        Get CPU utilization percentage
        """
        avg_time = self.get_average_processing_time()
        return (avg_time / self.target_period) * 100.0
```

### 2. Computational Efficiency
Optimize computational efficiency:

```python
class EfficientFusionAlgorithms:
    """
    Computationally efficient fusion algorithms
    """
    def __init__(self):
        # Pre-computed constants and matrices
        self.identity_matrix = np.eye(12, dtype=np.float64)
        self.zeros_matrix = np.zeros((12, 12), dtype=np.float64)

        # Specialized algorithms for different scenarios
        self.algorithms = {
            'simple_kf': self._simple_kf_update,
            'ekf': self._ekf_update,
            'ukf': self._ukf_update,
            'particle': self._particle_filter_update
        }

    def _simple_kf_update(self, state, covariance, measurement, H, R):
        """
        Optimized Kalman filter update for linear systems
        """
        # Innovation
        innovation = measurement - H @ state

        # Innovation covariance
        S = H @ covariance @ H.T + R

        # Kalman gain
        K = covariance @ H.T @ np.linalg.inv(S)

        # Update state
        state_new = state + K @ innovation

        # Update covariance (Joseph form for stability)
        I_KH = self.identity_matrix - K @ H
        P_new = I_KH @ covariance @ I_KH.T + K @ R @ K.T

        return state_new, P_new

    def _batch_process_measurements(self, measurements, H_matrix, R_matrix):
        """
        Process multiple measurements in batch for efficiency
        """
        # Stack measurements
        stacked_measurements = np.vstack(measurements)

        # Batch innovation calculation
        expected_measurements = H_matrix @ self.state_vector
        innovations = stacked_measurements - expected_measurements

        # Batch covariance update
        S = H_matrix @ self.covariance_matrix @ H_matrix.T + R_matrix

        # Batch Kalman gain
        K = self.covariance_matrix @ H_matrix.T @ np.linalg.inv(S)

        # Batch update
        self.state_vector += K @ np.mean(innovations, axis=0)

        return self.state_vector

    def use_sparse_matrices(self, large_covariance):
        """
        Use sparse matrices when appropriate
        """
        import scipy.sparse as sp

        # Convert to sparse if beneficial
        sparse_cov = sp.csr_matrix(large_covariance)

        # Perform operations with sparse matrices
        # Only use if matrix is actually sparse (mostly zeros)
        if sp.issparse(sparse_cov) and sparse_cov.nnz < sparse_cov.size * 0.1:
            return sparse_cov
        else:
            return large_covariance  # Keep as dense
```

## Validation and Testing

### 1. Comprehensive Testing Framework
Implement thorough testing for your fusion system:

```python
import unittest
import numpy as np
from scipy import stats

class FusionSystemTests(unittest.TestCase):
    """
    Comprehensive tests for sensor fusion system
    """
    def setUp(self):
        self.fusion_system = ModularSensorFusion()

    def test_state_consistency(self):
        """
        Test that state estimates remain consistent
        """
        initial_state = self.fusion_system.state_estimator.state_vector.copy()

        # Process some data
        for _ in range(100):
            sensor_data = self._generate_test_data()
            self.fusion_system.process_sensor_data(sensor_data)

        final_state = self.fusion_system.state_estimator.state_vector

        # Check that state remains finite
        self.assertTrue(np.all(np.isfinite(final_state)))

        # Check that covariance remains positive definite
        eigenvals = np.linalg.eigvals(self.fusion_system.state_estimator.covariance)
        self.assertTrue(np.all(eigenvals > 0))

    def test_sensor_failure_tolerance(self):
        """
        Test system behavior when sensors fail
        """
        robust_system = RobustFusionSystem()

        # Add sensors
        robust_system.register_sensor('imu', MockSensor(healthy=True), None)
        robust_system.register_sensor('lidar', MockSensor(healthy=False), None)  # Failed sensor

        # Process data - should handle failed sensor gracefully
        result = robust_system.process_with_health_check(time.time())

        # Should still produce valid output despite one failed sensor
        self.assertIsNotNone(result)

    def test_outlier_rejection(self):
        """
        Test outlier rejection capabilities
        """
        outlier_system = OutlierRejectionFusion()

        # Create measurement with outlier
        normal_measurement = np.array([1.0, 2.0, 3.0])
        outlier_measurement = np.array([1.0, 2.0, 100.0])  # Large outlier in z

        # Normal measurement should be accepted
        result_normal = outlier_system.update_with_outlier_rejection(
            normal_measurement, np.eye(3), np.eye(3)
        )
        self.assertTrue(result_normal)

        # Outlier measurement should be rejected
        result_outlier = outlier_system.update_with_outlier_rejection(
            outlier_measurement, np.eye(3), np.eye(3)
        )
        self.assertFalse(result_outlier)

    def test_numerical_stability(self):
        """
        Test numerical stability over long runs
        """
        stable_system = NumericallyStableFusion()

        # Run many iterations
        for _ in range(1000):
            # Simulate measurement update
            measurement = np.random.normal(0, 1, 6)
            H = np.random.normal(0, 1, (6, 12))
            R = np.eye(6)

            stable_system.update_with_cholesky(measurement, H, R)
            stable_system.enforce_symmetry()
            stable_system.ensure_positive_definite()

        # Check that everything remains well-behaved
        self.assertTrue(np.all(np.isfinite(stable_system.state_vector)))
        self.assertTrue(np.all(np.isfinite(stable_system.covariance_matrix)))
        self.assertTrue(np.allclose(
            stable_system.covariance_matrix,
            stable_system.covariance_matrix.T
        ))  # Should be symmetric

    def _generate_test_data(self):
        """
        Generate realistic test sensor data
        """
        return {
            'imu': np.random.normal(0, 0.1, 6),  # [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
            'lidar': np.random.normal(0, 0.02, 1081),  # Typical LiDAR points
            'camera': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)  # Image
        }

class MockSensor:
    """
    Mock sensor for testing
    """
    def __init__(self, healthy=True):
        self.healthy = healthy

    def get_measurement(self):
        if self.healthy:
            return np.random.normal(0, 1, 6)
        else:
            # Return invalid measurement to simulate failure
            return np.array([np.nan, np.nan, np.nan, np.nan, np.nan, np.nan])

# Performance testing
class PerformanceTests:
    """
    Performance tests for sensor fusion system
    """
    def __init__(self):
        self.fusion_system = ModularSensorFusion()

    def test_real_time_performance(self):
        """
        Test if system can maintain real-time performance
        """
        import time

        start_time = time.time()
        iterations = 1000

        for _ in range(iterations):
            sensor_data = {
                'imu': np.random.normal(0, 0.1, 6),
                'lidar': np.random.normal(0, 0.02, 100),
                'camera': np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
            }

            self.fusion_system.process_sensor_data(sensor_data)

        total_time = time.time() - start_time
        avg_time = total_time / iterations
        frequency = 1.0 / avg_time if avg_time > 0 else 0

        print(f"Average processing time: {avg_time*1000:.2f} ms")
        print(f"Average frequency: {frequency:.2f} Hz")

        # Should achieve at least 50 Hz for real-time operation
        self.assertGreater(frequency, 50, "System should maintain >50 Hz")
```

## Configuration and Tuning

### 1. Parameter Management
Properly manage fusion parameters:

```python
import json
from pathlib import Path

class FusionParameterManager:
    """
    Manage fusion system parameters
    """
    def __init__(self, config_file=None):
        self.parameters = {
            'process_noise': np.diag([0.1, 0.1, 0.2, 0.01, 0.01, 0.02, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1]),
            'measurement_noise': {
                'imu': np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001]),
                'lidar': np.diag([0.1, 0.1, 0.1]),
                'camera': np.diag([2.0, 2.0])  # 2 pixel uncertainty
            },
            'outlier_threshold': 3.0,
            'innovation_threshold': 3.0,
            'sensor_weights': {'imu': 1.0, 'lidar': 1.0, 'camera': 0.8},
            'fusion_frequency': 100,
            'buffer_size': 100
        }

        if config_file:
            self.load_parameters(config_file)

    def load_parameters(self, config_file):
        """
        Load parameters from configuration file
        """
        with open(config_file, 'r') as f:
            config = json.load(f)

        # Update parameters from config
        for key, value in config.items():
            if key in self.parameters:
                if isinstance(self.parameters[key], np.ndarray):
                    self.parameters[key] = np.array(value)
                else:
                    self.parameters[key] = value

    def save_parameters(self, config_file):
        """
        Save parameters to configuration file
        """
        # Convert numpy arrays to lists for JSON serialization
        serializable_params = {}
        for key, value in self.parameters.items():
            if isinstance(value, np.ndarray):
                serializable_params[key] = value.tolist()
            else:
                serializable_params[key] = value

        with open(config_file, 'w') as f:
            json.dump(serializable_params, f, indent=2)

    def auto_tune_parameters(self, training_data):
        """
        Auto-tune parameters based on training data
        """
        # Example: estimate noise parameters from data
        imu_data = training_data.get('imu', [])
        if imu_data:
            # Estimate IMU noise from stationary periods
            imu_noise = self._estimate_noise_from_stationary(imu_data)
            self.parameters['measurement_noise']['imu'] = imu_noise

    def _estimate_noise_from_stationary(self, data):
        """
        Estimate sensor noise from stationary periods
        """
        # Calculate standard deviation of stationary data
        data_array = np.array(data)
        noise_std = np.std(data_array, axis=0)

        # Create covariance matrix
        return np.diag(noise_std**2)

    def validate_parameters(self):
        """
        Validate that parameters are reasonable
        """
        errors = []

        # Check process noise is positive
        if np.any(np.diag(self.parameters['process_noise']) <= 0):
            errors.append("Process noise must be positive")

        # Check measurement noise is positive
        for sensor, noise in self.parameters['measurement_noise'].items():
            if np.any(np.diag(noise) <= 0):
                errors.append(f"Measurement noise for {sensor} must be positive")

        # Check outlier threshold is reasonable
        if self.parameters['outlier_threshold'] <= 0:
            errors.append("Outlier threshold must be positive")

        return errors
```

## Best Practices Summary

### Key Recommendations:

1. **Start Simple**: Begin with basic fusion algorithms and gradually add complexity
2. **Validate Continuously**: Implement validation at every level of your system
3. **Handle Edge Cases**: Plan for sensor failures, outliers, and unusual operating conditions
4. **Monitor Performance**: Continuously monitor system performance and adjust parameters
5. **Document Assumptions**: Clearly document all assumptions and limitations
6. **Test Thoroughly**: Implement comprehensive testing including unit, integration, and performance tests
7. **Optimize Gradually**: Profile your system and optimize bottlenecks as needed
8. **Plan for Maintenance**: Design systems that are easy to update and maintain

### Common Pitfalls to Avoid:

- Overcomplicating the initial design
- Ignoring numerical stability issues
- Failing to handle sensor failures gracefully
- Not validating assumptions about sensor characteristics
- Poor parameter tuning
- Inadequate testing of edge cases
- Ignoring computational constraints

These best practices provide a solid foundation for implementing robust, efficient, and reliable sensor fusion systems in digital twin environments for humanoid robotics applications.