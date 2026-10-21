#ifndef MONTECARLO_LOCALIZATION_AMCL_ENGINE_HPP_
#define MONTECARLO_LOCALIZATION_AMCL_ENGINE_HPP_

#include <vector>
#include <unordered_set>
#include "montecarlo_localization/mcl_engine.hpp"

namespace montecarlo_localization
{

struct AMCLParameters : public MCLParameters
{
  int min_particles{500};
  int max_particles{5000};
  double kld_err{0.05};            // Error bound epsilon
  double kld_z{2.32635};           // Standard normal quantile z_{1-delta} (0.99 confidence)
  double spatial_resolution_x{0.5};     // Spatial bin size in X (meters)
  double spatial_resolution_y{0.5};     // Spatial bin size in Y (meters)
  double spatial_resolution_theta{0.17453}; // Angular bin size (radians, ~10 degrees)
};

/**
 * @brief 3D discrete grid bin key for KLD spatial binning (Fox 2001/2003)
 */
struct SpatialBinKey
{
  int bx;
  int by;
  int btheta;

  bool operator==(const SpatialBinKey & other) const
  {
    return bx == other.bx && by == other.by && btheta == other.btheta;
  }
};

struct SpatialBinHash
{
  std::size_t operator()(const SpatialBinKey & key) const
  {
    std::size_t h1 = std::hash<int>{}(key.bx);
    std::size_t h2 = std::hash<int>{}(key.by);
    std::size_t h3 = std::hash<int>{}(key.btheta);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

/**
 * @brief Adaptive Monte Carlo Localization Engine implementing Dieter Fox's
 *        KLD-Sampling algorithm (Fox 2001, 2003) combined with Augmented MCL recovery.
 */
class AMCLEngine : public MCLEngine
{
public:
  AMCLEngine();
  explicit AMCLEngine(const AMCLParameters & params);
  ~AMCLEngine() override = default;

  void set_amcl_parameters(const AMCLParameters & params);
  void set_parameters(const MCLParameters & params) override;
  const AMCLParameters & get_amcl_parameters() const {return amcl_params_;}

  /**
   * @brief Initialize particles around a Gaussian distribution with min_particles
   */
  void init_gaussian(
    double mean_x, double mean_y, double mean_yaw,
    double std_x, double std_y, double std_yaw) override;

  /**
   * @brief Initialize particles uniformly distributed across free map space with max_particles
   */
  bool init_global() override;

  void process_update(
    const Pose2D & prev_odom,
    const Pose2D & curr_odom,
    const sensor_msgs::msg::LaserScan & scan) override;

  void update_sensor(const sensor_msgs::msg::LaserScan & scan) override;

  /**
   * @brief KLD-Sampling Resampling (Dieter Fox 2001/2003):
   *        Adapts sample size M based on statistical dispersion in the state space.
   */
  void resample() override;

private:
  AMCLParameters amcl_params_;
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_AMCL_ENGINE_HPP_
