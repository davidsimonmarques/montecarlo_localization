#ifndef MONTECARLO_LOCALIZATION_MHMCL_ENGINE_HPP_
#define MONTECARLO_LOCALIZATION_MHMCL_ENGINE_HPP_

#include "montecarlo_localization/mcl_engine.hpp"

namespace montecarlo_localization
{

/**
 * @brief MH-MCL: Monte Carlo Localization with Metropolis-Hastings Sampling
 *        Based on Villela, Farias & Viana (IEEE DASC 2025):
 *        "A Metropolis-Hastings Sampling Approach to Improve Monte Carlo Localization
 *         in ROS-Based Robotics"
 *
 * Algorithm 3 (MCL branch):
 * - Samples candidate particles from motion model
 * - Computes observation likelihood of proposed pose and previous pose
 * - Accepts or rejects proposed state using Metropolis-Hastings ratio alpha = min(1, w_prop / w_prev)
 * - Normalizes weights
 * - Executes simple resampling
 */
class MHMCLEngine : public MCLEngine
{
public:
  MHMCLEngine();
  explicit MHMCLEngine(const MCLParameters & params);
  ~MHMCLEngine() override = default;

  /**
   * @brief Executes one filter update iteration incorporating Metropolis-Hastings
   *        acceptance/rejection before resampling (Algorithm 3, lines 4-23).
   */
  void process_update(
    const Pose2D & prev_odom,
    const Pose2D & curr_odom,
    const sensor_msgs::msg::LaserScan & scan) override;

protected:
  /**
   * @brief Applies the Metropolis-Hastings sampling step across all particles.
   *        Returns the computed average observation likelihood.
   */
  virtual double apply_metropolis_hastings(
    const Pose2D & prev_odom,
    const Pose2D & curr_odom,
    const SensorModel::PreparedScan & prepared_scan);
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_MHMCL_ENGINE_HPP_
