#ifndef MONTECARLO_LOCALIZATION_SENSOR_MODEL_HPP_
#define MONTECARLO_LOCALIZATION_SENSOR_MODEL_HPP_

#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "montecarlo_localization/particle.hpp"
#include "montecarlo_localization/map_model.hpp"

namespace montecarlo_localization
{

struct SensorParameters
{
  double z_hit{0.85};         // Weight of hit component
  double z_rand{0.15};        // Weight of random noise component
  double sigma_hit{0.2};      // Standard deviation for obstacle hit Gaussian (meters)
  double min_range{0.1};      // Minimum valid range (meters)
  double max_range{10.0};     // Maximum valid range (meters)
  int beam_skip{4};           // Step to skip beams (subsampling for performance)
  int max_beams{60};          // Max number of beams evaluated per particle

  // Sensor mount offset on the robot (laser relative to base_link)
  double laser_x{0.0};
  double laser_y{0.0};
  double laser_yaw{0.0};
};

class SensorModel
{
public:
  SensorModel();
  explicit SensorModel(const SensorParameters & params);
  ~SensorModel() = default;

  void set_parameters(const SensorParameters & params) { params_ = params; }
  const SensorParameters & get_parameters() const { return params_; }

  void set_laser_offset(double x, double y, double yaw)
  {
    params_.laser_x = x;
    params_.laser_y = y;
    params_.laser_yaw = yaw;
  }

  /**
   * @brief Update particle weights using Likelihood Field model on the LaserScan
   * @param particles Particles to update
   * @param scan LaserScan message
   * @param map_model Precomputed distance grid map
   * @return Average observation likelihood across particles (before normalization)
   */
  double update_weights(
    std::vector<Particle> & particles,
    const sensor_msgs::msg::LaserScan & scan,
    const MapModel & map_model);

private:
  SensorParameters params_;
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_SENSOR_MODEL_HPP_
