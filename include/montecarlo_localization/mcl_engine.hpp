#ifndef MONTECARLO_LOCALIZATION_MCL_ENGINE_HPP_
#define MONTECARLO_LOCALIZATION_MCL_ENGINE_HPP_

#include <vector>
#include <random>
#include <array>
#include "montecarlo_localization/particle.hpp"
#include "montecarlo_localization/map_model.hpp"
#include "montecarlo_localization/motion_model.hpp"
#include "montecarlo_localization/sensor_model.hpp"
#include "montecarlo_localization/resampling.hpp"

namespace montecarlo_localization
{

struct MCLParameters
{
  int num_particles{500};
  double update_min_d{0.1};      // Minimum translational movement to trigger scan update (meters)
  double update_min_a{0.1};      // Minimum angular movement to trigger scan update (radians)
  bool resample_on_update{true}; // Resample immediately after each scan update

  MotionParameters motion_params;
  SensorParameters sensor_params;
  ResamplingParameters resample_params;
};

struct PoseEstimate
{
  Pose2D pose;
  std::array<double, 9> covariance;  // 3x3 covariance matrix: [cov(x,x), cov(x,y), cov(x,yaw), ...]
};

class MCLEngine
{
public:
  MCLEngine();
  explicit MCLEngine(const MCLParameters & params);
  ~MCLEngine() = default;

  void set_parameters(const MCLParameters & params);
  const MCLParameters & get_parameters() const { return params_; }

  /**
   * @brief Load occupancy grid map into map model
   */
  void set_map(const nav_msgs::msg::OccupancyGrid & map, int occupied_thresh = 50, int free_thresh = 20);

  /**
   * @brief Initialize particles around a Gaussian distribution
   */
  void init_gaussian(
    double mean_x, double mean_y, double mean_yaw,
    double std_x, double std_y, double std_yaw);

  /**
   * @brief Initialize particles uniformly distributed across free map space
   */
  bool init_global();

  /**
   * @brief Predict step: propagate particles using odometry displacement
   */
  void predict(const Pose2D & prev_odom, const Pose2D & curr_odom);

  /**
   * @brief Correction step: update particle weights using LaserScan observation
   */
  void update_sensor(const sensor_msgs::msg::LaserScan & scan);

  /**
   * @brief Resample particles
   */
  void resample();

  /**
   * @brief Check if robot has moved enough since last update pose
   */
  bool has_moved(const Pose2D & last_update_odom, const Pose2D & curr_odom) const;

  /**
   * @brief Compute the current best pose estimate and covariance
   */
  PoseEstimate get_estimate() const;

  /**
   * @brief Access the particle cloud
   */
  const std::vector<Particle> & get_particles() const { return particles_; }

  /**
   * @brief Check if MCL is initialized with particles and a valid map
   */
  bool is_initialized() const { return is_initialized_ && map_model_.is_loaded(); }

  // Access internal models
  MapModel & map_model() { return map_model_; }
  const MapModel & map_model() const { return map_model_; }
  MotionModel & motion_model() { return motion_model_; }
  SensorModel & sensor_model() { return sensor_model_; }
  Resampler & resampler() { return resampler_; }

private:
  MCLParameters params_;
  MapModel map_model_;
  MotionModel motion_model_;
  SensorModel sensor_model_;
  Resampler resampler_;

  std::vector<Particle> particles_;
  bool is_initialized_{false};
  std::mt19937 rng_;
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_MCL_ENGINE_HPP_
