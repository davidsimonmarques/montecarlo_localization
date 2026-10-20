# Monte Carlo Localization (MCL) - ROS 2

A ROS 2 (C++) package implementing the **Monte Carlo Localization** (Particle Filter) algorithm for differential drive mobile robots using 2D LiDAR scans and odometry on an occupancy grid map.

---

## Overview & Architecture

The algorithm probabilistically estimates the 2D robot pose $(x, y, \theta)$ in the map frame (`map`), eliminating the cumulative drift inherent to odometry.

### Execution Pipeline:
1. **Initialization:**
   * **Global Localization (`global_localization_on_start: true`):** Particles are uniformly distributed across free map cells only.
   * **Gaussian Initialization (`auto_initialize: true`):** Particles are sampled from a Gaussian distribution centered at a user-configured initial pose.
   * **Interactive:** Re-initialized at any time via `/initialpose` (e.g., using the *2D Pose Estimate* tool in RViz).
2. **Prediction (Motion Model):**
   * Odometry displacement is accumulated between filter steps. When the robot moves past `update_min_d` or `update_min_a`, the accumulated displacement $u_t = (\delta\text{rot}_1, \delta\text{trans}, \delta\text{rot}_2)$ propagates particles using the classical odometry motion model (*sample_motion_model_odometry*, Table 5.6) with Gaussian noise governed by $\alpha_1, \alpha_2, \alpha_3, \alpha_4$.
3. **Correction (Sensor Model - Likelihood Field):**
   * Projects valid beam endpoints of the current laser scan (`/scan`) into world coordinates and queries obstacle distances via a precomputed Euclidean Distance Field (EDF).
   * Computes measurement likelihood via Gaussian obstacle hit ($z_{\text{hit}}, \sigma_{\text{hit}}$) and uniform random noise ($z_{\text{rand}} / z_{\max}$) in log-space with log-sum-exp normalization (*likelihood_field_range_finder_model*, Table 6.3). Max-range readings are discarded.
4. **Resampling:**
   * Applies the classical Low-Variance Resampling algorithm (*low_variance_sampler*, Table 4.4) on each measurement update step, resetting all particle weights to $1 / M$.
5. **Pose Estimation & TF Broadcasting:**
   * The estimated pose is computed as the classical weighted sample mean of the particle set (with circular mean $\text{atan2}(\sum w_i \sin\theta_i, \sum w_i \cos\theta_i)$ for yaw) alongside its $3 \times 3$ covariance matrix.
   * The dynamic transform **`map -> odom`** is recomputed upon each sensor correction and re-broadcast with fresh timestamps on odometry ticks to keep TF listeners synchronized without artificial jitter.

---

## Topics & Transform (TF) Tree

### Subscribed Topics:
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Environment occupancy grid map (provided by `map_server`). |
| `/odom` | `nav_msgs/msg/Odometry` | Relative robot displacement and velocity from odometry. |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR range measurements. |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initial pose estimate sent manually from RViz. |

### Published Topics:
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/mcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Estimated robot pose $(x, y, \theta)$ with $6 \times 6$ covariance. |
| `/particlecloud` | `geometry_msgs/msg/PoseArray` | Sample of active particles for visualization in RViz. |

### Transform Tree (TF):
* **Required Inputs:** `odom -> base_link` (or `odom -> base_footprint -> base_link`) and `base_link -> base_scan`.
* **Broadcast by Node:** **`map -> odom`**.

---

## Configuration Files (`config/`)

* **[`params.yaml`](config/params.yaml):** Main configuration for the factory environment (`map_factory`):
  * `num_particles`: Active particle count (e.g., `8000`).
  * `update_min_d`, `update_min_a`: Minimum translation ($0.1\text{m}$) and rotation ($0.1\text{rad}$) thresholds required to trigger a sensor update.
  * `alpha1` to `alpha4`: Odometry motion model noise parameters.
  * `z_hit`, `z_rand`, `sigma_hit`: Likelihood field sensor model weights and standard deviation.
  * `global_localization_on_start`: `true` to disperse particles globally across the map, or `false` to use the initial pose.
  * `initial_pose_x`, `initial_pose_y`, `initial_pose_yaw`: Initial pose coordinates (in meters and **radians**).
* **[`amcl_params.yaml`](config/amcl_params.yaml):** Nav2 AMCL parameter file for benchmarking and performance comparisons.

---

## Launch Files (`launch/`)

* **`localization_launch.py`:**
  Launches the full localization pipeline for the factory environment:
  1. `nav2_map_server` loading `maps/map_factory.yaml`;
  2. `nav2_lifecycle_manager` managing lifecycle state transitions;
  3. `montecarlo_localization_node` using `config/params.yaml`;
  4. RViz2 with pre-configured display settings.
  ```bash
  ros2 launch montecarlo_localization localization_launch.py
  ```

* **`amcl_launch.py`:**
  Runs standard Nav2 AMCL on the same factory map for side-by-side evaluation:
  ```bash
  ros2 launch montecarlo_localization amcl_launch.py
  ```

---

## RViz Visualization (`rviz/`)

The pre-configured layout **[`rviz/mcl_visualization.rviz`](rviz/mcl_visualization.rviz)** provides:
* **Fixed Frame:** `map`
* **Map Display:** `/map` rendered on the ground plane.
* **LaserScan Display:** `/scan` colored by intensity or distance.
* **TF Display:** Full coordinate frame tree (`map`, `odom`, `base_link`, `base_scan`).
* **Robot Pose:** `/mcl_pose` depicted as an arrow showing the current estimate.
* **Particle Cloud:** `/particlecloud` displaying the particle dispersion in real time.
* **Ground Truth (if published):** `/pose_ground_truth` for immediate visual comparison between estimation and simulation ground truth.

---

## Build and Execution

Inside your ROS 2 workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select montecarlo_localization --symlink-install
source install/setup.bash
ros2 launch montecarlo_localization localization_launch.py
```
