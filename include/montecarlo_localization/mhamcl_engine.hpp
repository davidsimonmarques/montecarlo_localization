#ifndef MONTECARLO_LOCALIZATION_MHAMCL_ENGINE_HPP_
#define MONTECARLO_LOCALIZATION_MHAMCL_ENGINE_HPP_

#include <unordered_set>
#include "montecarlo_localization/mhmcl_engine.hpp"
#include "montecarlo_localization/amcl_engine.hpp"

namespace montecarlo_localization
{

/**
 * @brief MH-AMCL: Adaptive Monte Carlo Localization with Metropolis-Hastings Sampling
 *        Based on Villela, Farias & Viana (IEEE DASC 2025):
 *        "A Metropolis-Hastings Sampling Approach to Improve Monte Carlo Localization
 *         in ROS-Based Robotics"
 *
 * Algorithm 3 (AMCL branch):
 * - Samples candidate particles from motion model
 * - Evaluates observation likelihoods of proposed and previous poses
 * - Accepts or rejects proposed state using Metropolis-Hastings ratio alpha = min(1, w_prop / w_prev)
 * - Normalizes weights
 * - Updates slow and fast average likelihoods (w_slow, w_fast)
 * - Executes Mixed Resampling with random sample recovery injection rate rho = max(0, 1 - w_fast / w_slow)
 *   and adaptive KLD-sampling sample size determination.
 */
class MHAMCLEngine : public MHMCLEngine
{
public:
  MHAMCLEngine();
  explicit MHAMCLEngine(const AMCLParameters & params);
  ~MHAMCLEngine() override = default;

  void set_amcl_parameters(const AMCLParameters & params);
  const AMCLParameters & get_amcl_parameters() const { return amcl_params_; }

  void init_gaussian(
    double mean_x, double mean_y, double mean_yaw,
    double std_x, double std_y, double std_yaw) override;

  bool init_global() override;

  /**
   * @brief Mixed Resampling with recovery (Algorithm 3, lines 17-22) and KLD-sampling.
   */
  void resample() override;

private:
  AMCLParameters amcl_params_;
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_MHAMCL_ENGINE_HPP_
