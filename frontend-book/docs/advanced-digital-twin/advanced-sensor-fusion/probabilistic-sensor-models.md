---
sidebar_position: 21
---

# Probabilistic Sensor Models

Probabilistic sensor models form the theoretical foundation for robust sensor fusion in digital twin environments. This chapter explores how to model sensor uncertainties, noise characteristics, and measurement processes using probabilistic frameworks that enable reliable state estimation for humanoid robots.

## Understanding Probabilistic Sensor Models

Probabilistic sensor models describe the relationship between the true state of the environment and sensor measurements through probability distributions. Rather than providing deterministic measurements, these models capture the inherent uncertainty in sensor readings, which is crucial for digital twin environments where simulated sensors must behave realistically.

### Mathematical Framework

The probabilistic sensor model is typically expressed as:

```
P(z | x, m)
```

Where:
- `z` is the sensor measurement
- `x` is the robot's state (position, orientation, etc.)
- `m` is the map or environmental state
- `P(z | x, m)` is the probability of observing measurement `z` given state `x` and map `m`

This probabilistic approach allows us to quantify uncertainty and make informed decisions when fusing information from multiple sensors with different characteristics.

## Gaussian Noise Models

Gaussian (normal) distributions are commonly used to model sensor noise due to their mathematical tractability and the Central Limit Theorem:

```python
import numpy as np
from scipy.stats import norm, multivariate_normal
import matplotlib.pyplot as plt

class GaussianSensorModel:
    def __init__(self, mean=0.0, std_dev=1.0):
        """
        Simple Gaussian sensor model
        :param mean: Expected sensor reading when true value is accurate
        :param std_dev: Standard deviation of sensor noise
        """
        self.mean = mean
        self.std_dev = std_dev

    def probability(self, true_value, observed_value):
        """
        Calculate probability of observing a value given the true value
        """
        # Adjust mean to be centered on true value
        mean = self.mean + true_value
        return norm.pdf(observed_value, loc=mean, scale=self.std_dev)

    def sample(self, true_value, num_samples=1):
        """
        Generate sensor readings given a true value
        """
        noise = np.random.normal(self.mean, self.std_dev, num_samples)
        return true_value + noise

    def log_probability(self, true_value, observed_value):
        """
        Calculate log probability (useful for numerical stability)
        """
        mean = self.mean + true_value
        return norm.logpdf(observed_value, loc=mean, scale=self.std_dev)

# Example usage for distance sensor (e.g., sonar)
def example_gaussian_model():
    # Model a distance sensor with 2cm standard deviation
    distance_sensor = GaussianSensorModel(mean=0.0, std_dev=0.02)  # 2cm = 0.02m

    true_distance = 1.5  # 1.5 meters
    observed_distance = 1.48  # Sensor reads 1.48 meters

    # Calculate probability of this observation
    prob = distance_sensor.probability(true_distance, observed_distance)
    print(f"Probability of observing {observed_distance}m when true distance is {true_distance}m: {prob:.6f}")

    # Generate multiple samples to see distribution
    samples = distance_sensor.sample(true_distance, num_samples=1000)
    print(f"Sample mean: {np.mean(samples):.3f}m, Sample std: {np.std(samples):.3f}m")
    print(f"True distance: {true_distance}m")

# Multi-dimensional Gaussian model for sensors with multiple outputs
class MultiDimensionalGaussianModel:
    def __init__(self, mean_vector, covariance_matrix):
        """
        Multi-dimensional Gaussian sensor model
        :param mean_vector: Expected values for each measurement dimension
        :param covariance_matrix: Covariance matrix describing correlations between dimensions
        """
        self.mean_vector = np.array(mean_vector)
        self.covariance_matrix = np.array(covariance_matrix)
        self.dim = len(mean_vector)

    def probability(self, true_state, observed_state):
        """
        Calculate probability of observing a multi-dimensional state
        """
        # Adjust mean to be centered on true state
        adjusted_mean = self.mean_vector + true_state
        return multivariate_normal.pdf(observed_state, mean=adjusted_mean, cov=self.covariance_matrix)

    def sample(self, true_state, num_samples=1):
        """
        Generate multi-dimensional sensor readings
        """
        noise_samples = np.random.multivariate_normal(self.mean_vector, self.covariance_matrix, num_samples)
        if num_samples == 1:
            return true_state + noise_samples[0]
        return true_state + noise_samples

    def update_covariance(self, new_covariance):
        """
        Update the covariance matrix (useful for adaptive models)
        """
        self.covariance_matrix = new_covariance

# Example: IMU sensor model with correlated noise
def example_multidimensional_gaussian():
    # IMU model: [acceleration_x, acceleration_y, acceleration_z, angular_rate_x, angular_rate_y, angular_rate_z]
    mean_vector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # No bias
    # Covariance matrix - off-diagonal elements show correlations between axes
    covariance_matrix = np.array([
        [0.01, 0.001, 0.001, 0.0, 0.0, 0.0],     # Accel X
        [0.001, 0.01, 0.001, 0.0, 0.0, 0.0],     # Accel Y
        [0.001, 0.001, 0.02, 0.0, 0.0, 0.0],     # Accel Z (higher noise due to gravity)
        [0.0, 0.0, 0.0, 0.0001, 0.0, 0.0],       # Angular rate X
        [0.0, 0.0, 0.0, 0.0, 0.0001, 0.0],       # Angular rate Y
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]        # Angular rate Z
    ])

    imu_model = MultiDimensionalGaussianModel(mean_vector, covariance_matrix)

    # True IMU state [ax, ay, az, wx, wy, wz]
    true_imu_state = np.array([0.1, 0.05, 9.81, 0.01, 0.02, 0.0])  # Including gravity

    # Observed IMU state (with noise)
    observed_imu_state = np.array([0.098, 0.052, 9.805, 0.011, 0.019, 0.001])

    # Calculate probability of observation
    prob = imu_model.probability(true_imu_state, observed_imu_state)
    print(f"Probability of IMU observation: {prob:.10f}")

    # Generate multiple samples
    samples = imu_model.sample(true_imu_state, num_samples=5)
    print(f"Generated IMU samples:")
    for i, sample in enumerate(samples):
        print(f"  Sample {i+1}: {sample}")

if __name__ == "__main__":
    example_gaussian_model()
    print()
    example_multidimensional_gaussian()
```

## Ray Casting Models for Range Sensors

Ray casting models are particularly useful for range sensors like LiDAR, where the sensor emits rays and measures distances to obstacles:

```python
import numpy as np
from scipy.spatial.distance import cdist

class RayCastingSensorModel:
    def __init__(self, max_range=30.0, min_range=0.1, resolution=0.01,
                 lambda_short=0.01, lambda_free=0.001, z_hit=0.8, z_short=0.1,
                 z_max=0.05, z_rand=0.05):
        """
        Probabilistic ray casting model for range sensors
        :param max_range: Maximum sensor range
        :param min_range: Minimum sensor range
        :param resolution: Map resolution
        :param lambda_short: Exponential distribution parameter for short readings
        :param lambda_free: Exponential distribution parameter for free space
        :param z_hit: Probability of hit component
        :param z_short: Probability of short reading component
        :param z_max: Probability of maximum range component
        :param z_rand: Probability of random reading component
        """
        self.max_range = max_range
        self.min_range = min_range
        self.resolution = resolution
        self.lambda_short = lambda_short
        self.lambda_free = lambda_free
        self.z_hit = z_hit
        self.z_short = z_short
        self.z_max = z_max
        self.z_rand = z_rand

    def calculate_likelihood(self, sensor_reading, expected_distance, sigma_hit=0.5):
        """
        Calculate likelihood of sensor reading given expected distance
        :param sensor_reading: Actual sensor reading
        :param expected_distance: Expected distance to obstacle
        :param sigma_hit: Standard deviation for hit component
        :return: Likelihood value
        """
        if sensor_reading >= self.max_range:
            # Maximum range reading
            if expected_distance >= self.max_range:
                return self.z_max + self.z_rand * 1.0
            else:
                return self.z_max
        elif sensor_reading < self.min_range:
            # Out of range below minimum
            return 1.0
        elif sensor_reading > expected_distance:
            # Unexpected free space (sensor reading > expected)
            return self.z_rand * (1.0 / self.max_range) + self.z_max
        else:
            # Short reading (sensor reading < expected) - obstacles or noise
            p_hit = self._gaussian_component(sensor_reading, expected_distance, sigma_hit)
            p_short = self._exponential_component(sensor_reading)
            p_rand = self.z_rand * (1.0 / self.max_range)

            return self.z_hit * p_hit + self.z_short * p_short + self.z_max + p_rand

    def _gaussian_component(self, z, z_expected, sigma):
        """Gaussian component of the sensor model"""
        return (1.0 / (sigma * np.sqrt(2 * np.pi))) * \
               np.exp(-0.5 * ((z - z_expected) / sigma) ** 2)

    def _exponential_component(self, z):
        """Exponential component for short readings (due to unexpected obstacles)"""
        return self.lambda_short * np.exp(-self.lambda_short * z)

    def batch_likelihood(self, sensor_readings, expected_distances, sigma_hit=0.5):
        """
        Calculate likelihood for multiple sensor readings at once
        :param sensor_readings: Array of sensor readings
        :param expected_distances: Array of expected distances
        :param sigma_hit: Standard deviation for hit component
        :return: Array of likelihoods
        """
        likelihoods = np.zeros(len(sensor_readings))

        for i in range(len(sensor_readings)):
            likelihoods[i] = self.calculate_likelihood(
                sensor_readings[i], expected_distances[i], sigma_hit
            )

        return likelihoods

# Implementation for grid map-based sensor simulation
class GridMapSensorSimulator:
    def __init__(self, occupancy_grid, resolution=0.05, origin=(0, 0)):
        """
        Simulate sensor readings based on occupancy grid
        :param occupancy_grid: 2D array representing occupancy probabilities
        :param resolution: Size of each grid cell in meters
        :param origin: (x, y) position of grid origin in world coordinates
        """
        self.grid = occupancy_grid
        self.resolution = resolution
        self.origin_x, self.origin_y = origin
        self.height, self.width = occupancy_grid.shape

    def simulate_range_scan(self, robot_pose, angles, max_range=30.0):
        """
        Simulate range scan from robot pose
        :param robot_pose: (x, y, theta) robot position and orientation
        :param angles: Array of angles for each ray in the scan
        :param max_range: Maximum sensor range
        :return: Array of simulated range readings
        """
        x, y, theta = robot_pose
        ranges = []

        for angle in angles:
            # Calculate world coordinates for ray
            world_angle = theta + angle
            range_reading = self._cast_ray(x, y, world_angle, max_range)
            ranges.append(range_reading)

        return np.array(ranges)

    def _cast_ray(self, start_x, start_y, angle, max_range):
        """
        Cast a single ray and return the distance to the first obstacle
        """
        # Convert to grid coordinates
        grid_start_x = int((start_x - self.origin_x) / self.resolution)
        grid_start_y = int((start_y - self.origin_y) / self.resolution)

        # Ray direction
        dx = np.cos(angle)
        dy = np.sin(angle)

        # Step along the ray
        step_size = self.resolution
        distance = 0.0

        while distance <= max_range:
            # Calculate current position
            curr_x = start_x + distance * dx
            curr_y = start_y + distance * dy

            # Convert to grid coordinates
            grid_x = int((curr_x - self.origin_x) / self.resolution)
            grid_y = int((curr_y - self.origin_y) / self.resolution)

            # Check bounds
            if (grid_x < 0 or grid_x >= self.width or
                grid_y < 0 or grid_y >= self.height):
                return max_range  # Out of map bounds

            # Check if cell is occupied (typically > 0.5)
            if self.grid[grid_y, grid_x] > 0.5:
                return distance  # Found obstacle

            distance += step_size

        return max_range  # No obstacle found within range

# Example usage with a simple occupancy grid
def example_ray_casting():
    # Create a simple occupancy grid (100x100 cells, 0.1m resolution)
    occupancy_grid = np.zeros((100, 100))

    # Add some obstacles (walls)
    occupancy_grid[20:80, 49:51] = 0.9  # Vertical wall in the middle
    occupancy_grid[49:51, 20:80] = 0.9  # Horizontal wall in the middle

    # Create sensor simulator
    sensor_sim = GridMapSensorSimulator(occupancy_grid, resolution=0.1, origin=(-5, -5))

    # Robot at center of grid (0, 0) facing east (0 degrees)
    robot_pose = (0.0, 0.0, 0.0)

    # 360-degree laser scan every 1 degree
    angles = np.deg2rad(np.arange(0, 360, 1))

    # Simulate range scan
    simulated_ranges = sensor_sim.simulate_range_scan(robot_pose, angles, max_range=30.0)

    print(f"Simulated {len(simulated_ranges)} range measurements")
    print(f"Minimum range: {np.min(simulated_ranges):.2f}m")
    print(f"Maximum range: {np.max(simulated_ranges):.2f}m")
    print(f"Mean range: {np.mean(simulated_ranges):.2f}m")

    # Apply probabilistic model to the simulated readings
    ray_model = RayCastingSensorModel()

    # For this example, let's say we have expected distances for each ray
    # (in a real system, these would come from your map)
    expected_distances = np.full_like(simulated_ranges, 30.0)  # Assume far objects

    # Add some variation to expected distances for more realistic scenario
    expected_distances[:90] = 5.0   # Objects at 5m in front-left
    expected_distances[90:270] = 10.0  # Objects at 10m on the sides
    expected_distances[270:] = 5.0  # Objects at 5m in front-right

    # Calculate likelihoods
    likelihoods = ray_model.batch_likelihood(simulated_ranges, expected_distances)

    print(f"Mean likelihood: {np.mean(likelihoods):.6f}")
    print(f"Min likelihood: {np.min(likelihoods):.6f}")
    print(f"Max likelihood: {np.max(likelihoods):.6f}")

if __name__ == "__main__":
    example_ray_casting()
```

## Particle Filter Integration

Particle filters naturally integrate with probabilistic sensor models by weighting particles based on the likelihood of sensor observations:

```python
class ParticleFilter:
    def __init__(self, num_particles=1000, state_dim=3):
        """
        Particle filter for robot localization
        :param num_particles: Number of particles to use
        :param state_dim: Dimension of the state vector (x, y, theta, etc.)
        """
        self.num_particles = num_particles
        self.state_dim = state_dim

        # Initialize particles uniformly or based on prior knowledge
        self.particles = np.random.uniform(-10, 10, (num_particles, state_dim))
        self.weights = np.ones(num_particles) / num_particles

        # Motion model noise
        self.motion_noise = np.array([0.1, 0.1, 0.05])  # [dx, dy, dtheta]

        # Sensor model for likelihood calculation
        self.sensor_model = RayCastingSensorModel()

    def predict(self, control_input, dt=1.0):
        """
        Prediction step: move particles according to motion model
        :param control_input: Control vector [linear_velocity, angular_velocity]
        :param dt: Time step
        """
        v, omega = control_input

        # Add noise to motion model
        noise = np.random.normal(0, self.motion_noise, (self.num_particles, self.state_dim))

        # Update particle positions
        self.particles[:, 0] += (v * np.cos(self.particles[:, 2]) * dt) + noise[:, 0]  # x
        self.particles[:, 1] += (v * np.sin(self.particles[:, 2]) * dt) + noise[:, 1]  # y
        self.particles[:, 2] += (omega * dt) + noise[:, 2]  # theta

        # Normalize angles
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]),
                                         np.cos(self.particles[:, 2]))

    def update(self, sensor_reading, expected_distances):
        """
        Update step: reweight particles based on sensor observations
        :param sensor_reading: Actual sensor observations
        :param expected_distances: Expected distances from each particle's perspective
        """
        # Calculate likelihood for each particle
        for i in range(self.num_particles):
            # Calculate likelihood based on sensor model
            particle_likelihood = self.sensor_model.batch_likelihood(
                sensor_reading, expected_distances[i]
            ).mean()  # Average likelihood across all beams

            # Update particle's weight
            self.weights[i] *= particle_likelihood

        # Normalize weights
        if np.sum(self.weights) == 0:
            # Reset weights if they all become zero
            self.weights = np.ones(self.num_particles) / self.num_particles
        else:
            self.weights /= np.sum(self.weights)

        # Resample if effective sample size is too low
        effective_particles = 1.0 / np.sum(self.weights ** 2)
        if effective_particles < self.num_particles / 2.0:
            self._resample()

    def _resample(self):
        """
        Resample particles based on their weights
        """
        # Systematic resampling
        indices = self._systematic_resample()

        # Create new particles based on selected indices
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def _systematic_resample(self):
        """
        Systematic resampling algorithm
        """
        indices = np.zeros(self.num_particles, dtype=int)
        cumulative_sum = np.cumsum(self.weights)

        # Create uniform steps
        u = np.random.uniform(0, 1.0 / self.num_particles)
        i, j = 0, 0

        while i < self.num_particles:
            while cumulative_sum[j] < u:
                j += 1
            indices[i] = j
            u += 1.0 / self.num_particles
            i += 1

        return indices

    def estimate(self):
        """
        Calculate state estimate as weighted average of particles
        """
        # Weighted average of particles
        estimate = np.average(self.particles, axis=0, weights=self.weights)
        return estimate

    def get_uncertainty(self):
        """
        Calculate uncertainty based on particle distribution
        """
        # Calculate covariance matrix weighted by particle weights
        mean = self.estimate()
        diff = self.particles - mean
        weighted_diff = diff * self.weights[:, np.newaxis]
        covariance = weighted_diff.T @ diff

        # Calculate trace as a measure of overall uncertainty
        uncertainty = np.sqrt(np.trace(covariance))
        return uncertainty

# Example: Particle filter with probabilistic sensor models
def particle_filter_example():
    # Create particle filter
    pf = ParticleFilter(num_particles=1000, state_dim=3)  # x, y, theta

    # Simulate robot movement and sensor readings
    for t in range(100):
        # Control input: move forward with some rotation
        control_input = [0.5, 0.1]  # 0.5 m/s linear, 0.1 rad/s angular

        # Prediction step
        pf.predict(control_input, dt=0.1)

        # Simulate sensor reading (in practice, this would come from actual sensors)
        # For this example, we'll generate synthetic measurements
        num_beams = 360
        sensor_reading = np.random.uniform(0.5, 10.0, num_beams)  # Simulated ranges

        # Calculate expected distances for each particle (simplified)
        # In practice, this would involve ray casting from each particle's position
        expected_distances = np.array([
            np.full(num_beams, 10.0) + np.random.normal(0, 0.5, num_beams)
            for _ in range(pf.num_particles)
        ])

        # Update step
        pf.update(sensor_reading, expected_distances)

        # Print estimate periodically
        if t % 20 == 0:
            estimate = pf.estimate()
            uncertainty = pf.get_uncertainty()
            print(f"Step {t}: Estimated pose: [{estimate[0]:.3f}, {estimate[1]:.3f}, {estimate[2]:.3f}], "
                  f"Uncertainty: {uncertainty:.3f}")

if __name__ == "__main__":
    particle_filter_example()
```

## Occupancy Grid Mapping

Occupancy grid mapping uses probabilistic sensor models to build representations of the environment:

```python
class OccupancyGridMapper:
    def __init__(self, width=100, height=100, resolution=0.05,
                 p_occ=0.6, p_free=0.4, p_prior=0.5):
        """
        Occupancy grid mapping with probabilistic sensor models
        :param width: Grid width in cells
        :param height: Grid height in cells
        :param resolution: Size of each cell in meters
        :param p_occ: Probability that a cell is occupied when sensor says it is
        :param p_free: Probability that a cell is free when sensor says it is
        :param p_prior: Prior probability of occupancy (before any measurements)
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.p_occ = p_occ
        self.p_free = p_free
        self.p_prior = p_prior

        # Initialize log odds representation
        self.log_odds = np.log(self.p_prior / (1 - self.p_prior)) * np.ones((height, width))

        # Sensor model for range sensors
        self.range_sensor_model = RayCastingSensorModel()

        # Origin offset
        self.origin_x = -width * resolution / 2
        self.origin_y = -height * resolution / 2

    def update_from_scan(self, robot_pose, scan_ranges, angles):
        """
        Update occupancy grid from range scan
        :param robot_pose: Robot position (x, y, theta)
        :param scan_ranges: Range measurements from sensor
        :param angles: Angles corresponding to each range measurement
        """
        robot_x, robot_y, robot_theta = robot_pose

        # Convert robot position to grid coordinates
        grid_robot_x = int((robot_x - self.origin_x) / self.resolution)
        grid_robot_y = int((robot_y - self.origin_y) / self.resolution)

        for i, (range_reading, angle) in enumerate(zip(scan_ranges, angles)):
            # Calculate the world coordinates of the obstacle point
            world_angle = robot_theta + angle
            obstacle_x = robot_x + range_reading * np.cos(world_angle)
            obstacle_y = robot_y + range_reading * np.sin(world_angle)

            # Convert to grid coordinates
            grid_obs_x = int((obstacle_x - self.origin_x) / self.resolution)
            grid_obs_y = int((obstacle_y - self.origin_y) / self.resolution)

            # Bresenham's line algorithm to mark free space
            self._mark_free_space(grid_robot_x, grid_robot_y, grid_obs_x, grid_obs_y)

            # Update cell where obstacle was detected
            if (0 <= grid_obs_x < self.width and 0 <= grid_obs_y < self.height and
                range_reading < self.range_sensor_model.max_range):
                self._update_cell_occupancy(grid_obs_x, grid_obs_y, occupied=True)

    def _mark_free_space(self, x0, y0, x1, y1):
        """Use Bresenham's line algorithm to mark free space along the ray"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            # Don't update the cell where the obstacle was detected
            if x == x1 and y == y1:
                break

            if 0 <= x < self.width and 0 <= y < self.height:
                # Update as free space
                self._update_cell_occupancy(x, y, occupied=False)

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def _update_cell_occupancy(self, x, y, occupied):
        """Update the occupancy probability of a single cell"""
        # Calculate log odds ratio
        if occupied:
            # Cell is likely occupied
            l = np.log(self.p_occ / (1 - self.p_occ))
        else:
            # Cell is likely free
            l = np.log(self.p_free / (1 - self.p_free))

        # Update log odds (with bounds checking)
        self.log_odds[y, x] = np.clip(self.log_odds[y, x] + l, -10, 10)

    def get_probability_grid(self):
        """Convert log odds back to probability"""
        prob_grid = 1 - (1 / (1 + np.exp(self.log_odds)))
        return prob_grid

    def get_map_origin(self):
        """Get the world coordinates of the map origin"""
        return (self.origin_x, self.origin_y)

    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        world_x = grid_x * self.resolution + self.origin_x
        world_y = grid_y * self.resolution + self.origin_y
        return world_x, world_y

# Example usage
def occupancy_mapping_example():
    # Create mapper
    mapper = OccupancyGridMapper(width=200, height=200, resolution=0.1)

    # Simulate robot path and sensor readings
    for step in range(50):
        # Robot position (moving in a spiral pattern)
        angle = step * 0.2
        radius = 0.5 + step * 0.1
        robot_x = radius * np.cos(angle)
        robot_y = radius * np.sin(angle)
        robot_theta = angle + np.pi/2  # Robot facing outward

        # Simulate laser scan (simplified)
        num_beams = 360
        angles = np.linspace(0, 2*np.pi, num_beams, endpoint=False)

        # Simulate ranges with some obstacles
        ranges = np.full(num_beams, 10.0)  # Max range

        # Add some obstacles
        if step > 10:  # After robot moves a bit
            # Create a "wall" at a certain angle
            obstacle_start = int(45 * np.pi/180 / (2*np.pi/num_beams))
            obstacle_end = int(135 * np.pi/180 / (2*np.pi/num_beams))
            ranges[obstacle_start:obstacle_end] = 2.0  # Obstacle at 2m

        # Update map
        mapper.update_from_scan((robot_x, robot_y, robot_theta), ranges, angles)

        # Print progress
        if step % 10 == 0:
            print(f"Mapping step {step}: Robot at ({robot_x:.1f}, {robot_y:.1f})")

    # Get final probability grid
    prob_grid = mapper.get_probability_grid()

    print(f"Map shape: {prob_grid.shape}")
    print(f"Occupancy range: [{np.min(prob_grid):.3f}, {np.max(prob_grid):.3f}]")
    print(f"Mean occupancy: {np.mean(prob_grid):.3f}")

if __name__ == "__main__":
    occupancy_mapping_example()
```

## Adaptive Sensor Models

Real-world digital twins require sensor models that can adapt to changing conditions:

```python
class AdaptiveSensorModel:
    def __init__(self, initial_params=None):
        """
        Adaptive sensor model that updates parameters based on performance
        """
        if initial_params is None:
            self.params = {
                'std_dev': 0.1,  # Initial standard deviation
                'bias': 0.0,     # Initial bias
                'adaptive_factor': 0.01  # How quickly to adapt
            }
        else:
            self.params = initial_params.copy()

        # Performance tracking
        self.residuals = []  # Differences between predictions and measurements
        self.max_residual_history = 1000

    def update_params(self, prediction, measurement):
        """
        Update sensor model parameters based on prediction error
        """
        residual = abs(prediction - measurement)
        self.residuals.append(residual)

        # Keep only recent residuals
        if len(self.residuals) > self.max_residual_history:
            self.residuals.pop(0)

        # If we have enough data, adapt parameters
        if len(self.residuals) > 100:
            current_error = np.mean(self.residuals[-50:])  # Error on recent measurements
            historical_error = np.mean(self.residuals[:-50]) if len(self.residuals) > 50 else current_error

            # Adjust standard deviation based on error trends
            if current_error > historical_error * 1.5:  # Error is increasing
                self.params['std_dev'] *= 1.05  # Increase uncertainty
            elif current_error < historical_error * 0.8:  # Error is decreasing
                self.params['std_dev'] *= 0.95  # Decrease uncertainty (but don't go too low)
                self.params['std_dev'] = max(self.params['std_dev'], 0.01)

            # Adjust bias if there's a consistent offset
            recent_residuals = self.residuals[-50:]
            if len(recent_residuals) > 0:
                # Determine sign of bias based on recent measurements vs predictions
                pass  # Simplified for this example

    def probability(self, true_value, observed_value):
        """
        Calculate probability using current parameters
        """
        adjusted_mean = self.params['bias'] + true_value
        return norm.pdf(observed_value, loc=adjusted_mean, scale=self.params['std_dev'])

    def get_performance_metrics(self):
        """
        Get metrics about sensor model performance
        """
        if len(self.residuals) < 10:
            return {'error_mean': float('inf'), 'error_std': float('inf')}

        recent_errors = self.residuals[-100:]  # Last 100 measurements
        return {
            'error_mean': np.mean(recent_errors),
            'error_std': np.std(recent_errors),
            'num_observations': len(self.residuals)
        }

# Sensor model selector for multiple sensor types
class MultiSensorModel:
    def __init__(self):
        """
        Manages multiple probabilistic sensor models
        """
        self.models = {}
        self.weights = {}  # For sensor reliability tracking

    def add_sensor_model(self, sensor_name, model, initial_weight=1.0):
        """
        Add a sensor model to the collection
        :param sensor_name: Name of the sensor
        :param model: Probabilistic sensor model instance
        :param initial_weight: Initial reliability weight
        """
        self.models[sensor_name] = model
        self.weights[sensor_name] = initial_weight

    def get_fused_probability(self, sensor_observations, true_states):
        """
        Get fused probability from multiple sensors
        :param sensor_observations: Dict of sensor_name -> observed_value
        :param true_states: Dict of sensor_name -> true_value
        """
        fused_prob = 1.0
        total_weight = 0.0

        for sensor_name, obs_value in sensor_observations.items():
            if sensor_name in self.models and sensor_name in true_states:
                true_value = true_states[sensor_name]

                # Get individual sensor probability
                prob = self.models[sensor_name].probability(true_value, obs_value)

                # Weight by sensor reliability
                weight = self.weights[sensor_name]
                fused_prob *= (prob ** weight)
                total_weight += weight

        # Normalize by total weight
        if total_weight > 0:
            fused_prob = fused_prob ** (1.0 / total_weight)

        return fused_prob

    def update_sensor_reliability(self, sensor_observations, true_states, reference_value):
        """
        Update sensor reliability based on how well each sensor agrees with reference
        :param sensor_observations: Dict of sensor_name -> observed_value
        :param true_states: Dict of sensor_name -> true_value
        :param reference_value: Reference value to compare against
        """
        for sensor_name, obs_value in sensor_observations.items():
            if sensor_name in self.models:
                # Calculate error relative to reference
                error = abs(obs_value - reference_value)

                # Update sensor model parameters
                if hasattr(self.models[sensor_name], 'update_params'):
                    self.models[sensor_name].update_params(reference_value, obs_value)

                # Update reliability weight (lower error = higher reliability)
                max_error = 1.0  # Define what we consider maximum acceptable error
                reliability = max(0.1, 1.0 - (error / max_error))
                self.weights[sensor_name] = reliability

# Example of using multi-sensor models in digital twin
def multi_sensor_example():
    """Example combining multiple sensor models"""

    # Create sensor models
    lidar_model = AdaptiveSensorModel({'std_dev': 0.02, 'bias': 0.0, 'adaptive_factor': 0.01})
    camera_model = AdaptiveSensorModel({'std_dev': 0.05, 'bias': 0.01, 'adaptive_factor': 0.01})
    ultrasonic_model = AdaptiveSensorModel({'std_dev': 0.03, 'bias': -0.02, 'adaptive_factor': 0.01})

    # Create multi-sensor system
    sensor_system = MultiSensorModel()
    sensor_system.add_sensor_model('lidar', lidar_model, initial_weight=1.0)
    sensor_system.add_sensor_model('camera', camera_model, initial_weight=0.8)
    sensor_system.add_sensor_model('ultrasonic', ultrasonic_model, initial_weight=0.6)

    # Simulate sensor readings over time
    true_distance = 2.0  # 2 meters

    for t in range(1000):
        # Simulate sensor readings with different noise characteristics
        lidar_reading = np.random.normal(true_distance, 0.02)
        camera_reading = np.random.normal(true_distance, 0.05) + 0.01  # with bias
        ultrasonic_reading = np.random.normal(true_distance, 0.03) - 0.02  # with bias

        # Get fused probability
        observations = {
            'lidar': lidar_reading,
            'camera': camera_reading,
            'ultrasonic': ultrasonic_reading
        }

        true_states = {
            'lidar': true_distance,
            'camera': true_distance,
            'ultrasonic': true_distance
        }

        fused_prob = sensor_system.get_fused_probability(observations, true_states)

        # Update sensor reliability based on agreement with reference
        sensor_system.update_sensor_reliability(observations, true_states, true_distance)

        # Print progress periodically
        if t % 200 == 0:
            metrics = sensor_system.models['lidar'].get_performance_metrics()
            print(f"Step {t}: Fused probability: {fused_prob:.6f}")
            print(f"  Sensor weights: {sensor_system.weights}")
            print(f"  LiDAR metrics: {metrics}")
            print()

if __name__ == "__main__":
    multi_sensor_example()
```

## Implementation in Digital Twin Environment

Here's how probabilistic sensor models are implemented in a digital twin environment:

```python
# ROS/ROS2 node for probabilistic sensor models
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformListener
import tf2_geometry_msgs
import message_filters

class DigitalTwinProbabilisticSensors:
    def __init__(self):
        rospy.init_node('digital_twin_probabilistic_sensors')

        # Initialize probabilistic models for different sensors
        self.lidar_model = RayCastingSensorModel(
            max_range=30.0, min_range=0.1,
            z_hit=0.8, z_short=0.1, z_max=0.05, z_rand=0.05
        )

        self.imu_model = MultiDimensionalGaussianModel(
            mean_vector=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # No bias
            covariance_matrix=np.diag([0.01, 0.01, 0.02, 0.0001, 0.0001, 0.0001])  # Accel & gyro noise
        )

        self.adaptive_model = AdaptiveSensorModel({
            'std_dev': 0.02, 'bias': 0.0, 'adaptive_factor': 0.01
        })

        # Subscribers
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publishers for uncertainty estimates
        self.uncertainty_pub = rospy.Publisher('/sensor_uncertainty', Float32MultiArray, queue_size=10)
        self.probability_pub = rospy.Publisher('/sensor_probability', Float32MultiArray, queue_size=10)

        # TF listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Storage for sensor history and state estimation
        self.sensor_history = {
            'lidar': [],
            'imu': [],
            'odom': []
        }

        self.state_estimator = ParticleFilter(num_particles=1000, state_dim=6)  # x, y, z, roll, pitch, yaw

    def lidar_callback(self, msg):
        """Process LiDAR data with probabilistic model"""
        # Convert LaserScan to ranges
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max, neginf=0.0)

        # Apply probabilistic model to assess measurement quality
        # This is simplified - in practice you'd compare to expected values from map
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        if len(valid_ranges) > 0:
            # Calculate uncertainty based on number of valid measurements and their distribution
            uncertainty = 1.0 / (len(valid_ranges) + 1)  # More beams = less uncertainty

            # Update adaptive model based on recent performance
            # (Would require ground truth comparison in simulation)
            pass

        # Store for history
        self.sensor_history['lidar'].append({
            'timestamp': msg.header.stamp,
            'ranges': ranges,
            'uncertainty': float(uncertainty) if 'uncertainty' in locals() else 0.1
        })

        # Limit history size
        if len(self.sensor_history['lidar']) > 100:
            self.sensor_history['lidar'].pop(0)

    def imu_callback(self, msg):
        """Process IMU data with probabilistic model"""
        # Extract measurements
        linear_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        angular_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

        # Concatenate into single state vector for multi-dimensional model
        measurement = np.array(linear_acc + angular_vel)

        # Apply Gaussian model to assess measurement quality
        # This is simplified - would need true state for comparison
        imu_uncertainty = 0.01  # Base uncertainty

        # Store for history
        self.sensor_history['imu'].append({
            'timestamp': msg.header.stamp,
            'linear_acc': linear_acc,
            'angular_vel': angular_vel,
            'measurement': measurement,
            'uncertainty': imu_uncertainty
        })

        # Limit history size
        if len(self.sensor_history['imu']) > 100:
            self.sensor_history['imu'].pop(0)

    def odom_callback(self, msg):
        """Process odometry data"""
        # Extract position and orientation
        position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

        # Convert quaternion to euler (simplified)
        import tf.transformations as tf_trans
        orientation_q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler = tf_trans.euler_from_quaternion(orientation_q)

        # Extract linear and angular velocities
        linear_vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        angular_vel = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]

        # Store for history
        self.sensor_history['odom'].append({
            'timestamp': msg.header.stamp,
            'position': position,
            'orientation': euler,
            'linear_vel': linear_vel,
            'angular_vel': angular_vel,
            'covariance': list(msg.pose.covariance)
        })

        # Limit history size
        if len(self.sensor_history['odom']) > 100:
            self.sensor_history['odom'].pop(0)

    def publish_uncertainty_metrics(self):
        """Publish sensor uncertainty metrics"""
        if not all(self.sensor_history.values()):
            return

        # Calculate uncertainty metrics for each sensor
        uncertainty_msg = Float32MultiArray()
        probability_msg = Float32MultiArray()

        # Example uncertainty values (in practice, these would come from the probabilistic models)
        latest_lidar = self.sensor_history['lidar'][-1] if self.sensor_history['lidar'] else None
        latest_imu = self.sensor_history['imu'][-1] if self.sensor_history['imu'] else None

        uncertainties = []
        probabilities = []

        if latest_lidar:
            uncertainties.append(latest_lidar['uncertainty'])
            # Example probability calculation (would be more sophisticated in practice)
            probabilities.append(max(0.0, min(1.0, 1.0 - latest_lidar['uncertainty'])))

        if latest_imu:
            uncertainties.append(latest_imu['uncertainty'])
            probabilities.append(max(0.0, min(1.0, 1.0 - latest_imu['uncertainty'])))

        uncertainty_msg.data = uncertainties
        probability_msg.data = probabilities

        self.uncertainty_pub.publish(uncertainty_msg)
        self.probability_pub.publish(probability_msg)

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Publish uncertainty metrics
            self.publish_uncertainty_metrics()

            # Perform any sensor fusion or state estimation updates here
            # (Would integrate with Kalman filter or particle filter from other modules)

            rate.sleep()

def main():
    """Main function to run the probabilistic sensors node"""
    sensor_node = DigitalTwinProbabilisticSensors()

    try:
        sensor_node.run()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down probabilistic sensors node")

if __name__ == '__main__':
    main()
```

## Best Practices for Probabilistic Sensor Models

### 1. Model Validation
- Validate sensor models against ground truth data in simulation
- Use cross-validation techniques to ensure models generalize well
- Continuously monitor model performance metrics

### 2. Computational Efficiency
- Use efficient data structures for storing and accessing sensor history
- Implement early termination conditions where possible
- Consider approximate methods when exact computation is too expensive

### 3. Parameter Tuning
- Use maximum likelihood estimation to fit model parameters
- Implement automatic calibration procedures
- Regularly re-tune parameters based on performance

### 4. Robustness
- Handle sensor failures gracefully
- Implement fallback models when primary models fail
- Use multiple model hypotheses when uncertainty is high

Probabilistic sensor models are essential for creating realistic and robust digital twin environments. By properly modeling sensor uncertainties and incorporating them into fusion algorithms, you can achieve more accurate state estimation and better overall system performance in your humanoid robot simulations.