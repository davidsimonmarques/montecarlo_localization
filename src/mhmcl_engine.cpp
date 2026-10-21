#include "montecarlo_localization/mhmcl_engine.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <random>

namespace montecarlo_localization
{

MHMCLEngine::MHMCLEngine()
: MCLEngine()
{
}

MHMCLEngine::MHMCLEngine(const MCLParameters & params)
: MCLEngine(params)
{
}

void MHMCLEngine::process_update(
  const Pose2D & prev_odom,
  const Pose2D & curr_odom,
  const sensor_msgs::msg::LaserScan & scan)
{
  if (!is_initialized_ || particles_.empty() || !map_model_.is_loaded()) {
    return;
  }

  const SensorModel::PreparedScan prepared_scan = sensor_model_.prepare_scan(scan);
  const double avg_likelihood = apply_metropolis_hastings(prev_odom, curr_odom, prepared_scan);
  resampler_.update_recovery_weights(avg_likelihood);

  if (params_.resample_on_update) {
    resample();
  }
}

double MHMCLEngine::apply_metropolis_hastings(
  const Pose2D & prev_odom,
  const Pose2D & curr_odom,
  const SensorModel::PreparedScan & prepared_scan)
{
  const size_t n_particles = particles_.size();
  std::vector<double> log_weights(n_particles, -std::numeric_limits<double>::infinity());
  std::uniform_real_distribution<double> dist_u(0.0, 1.0);

  // Algorithm 3, lines 4-15: Metropolis-Hastings candidate evaluation
  for (size_t i = 0; i < n_particles; ++i) {
    const Particle prev_particle = particles_[i];
    Particle proposed = prev_particle;

    // Line 5: Sample proposed particle from motion model
    motion_model_.predict(proposed, prev_odom, curr_odom, rng_);

    // Line 6 & 7: Observation likelihoods of proposed and previous poses
    const double log_w_prop = sensor_model_.log_likelihood(proposed, prepared_scan, map_model_);
    const double log_w_prev = sensor_model_.log_likelihood(prev_particle, prepared_scan, map_model_);

    // Line 8: Compute acceptance probability alpha = min(1, w_prop / w_prev)
    double alpha = 0.0;
    if (!std::isfinite(log_w_prop)) {
      alpha = 0.0;
    } else if (!std::isfinite(log_w_prev)) {
      alpha = 1.0;
    } else {
      const double log_diff = log_w_prop - log_w_prev;
      alpha = log_diff >= 0.0 ? 1.0 : std::exp(log_diff);
    }

    // Lines 9-14: Accept or reject candidate
    if (dist_u(rng_) < alpha) {
      particles_[i] = proposed;
      log_weights[i] = log_w_prop;
    } else {
      particles_[i] = prev_particle;
      log_weights[i] = log_w_prev;
    }
  }

  // Line 16: Normalize particle weights using log-sum-exp
  double max_log = -std::numeric_limits<double>::infinity();
  for (const double lw : log_weights) {
    max_log = std::max(max_log, lw);
  }

  double average_likelihood = 0.0;
  if (std::isfinite(max_log)) {
    double sum_exp = 0.0;
    for (const double lw : log_weights) {
      if (std::isfinite(lw)) {
        sum_exp += std::exp(lw - max_log);
      }
    }

    const double log_normalizer = max_log + std::log(sum_exp);
    for (size_t i = 0; i < n_particles; ++i) {
      particles_[i].weight = std::isfinite(log_weights[i]) ?
        std::exp(log_weights[i] - log_normalizer) : 0.0;
    }

    const double log_avg = max_log + std::log(sum_exp) - std::log(static_cast<double>(n_particles));
    if (log_avg >= std::log(std::numeric_limits<double>::min())) {
      average_likelihood = std::exp(
        std::min(log_avg, std::log(std::numeric_limits<double>::max())));
    }
  } else {
    const double uniform_weight = 1.0 / static_cast<double>(n_particles);
    for (auto & p : particles_) {
      p.weight = uniform_weight;
    }
  }

  return average_likelihood;
}

}  // namespace montecarlo_localization
