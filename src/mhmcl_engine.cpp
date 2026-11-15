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
  update_count_++;

  bool should_resample = params_.resample_on_update &&
    (params_.resample_interval <= 1 || (update_count_ % params_.resample_interval == 0));

  if (should_resample && params_.selective_resampling) {
    const double n_eff = resampler_.compute_neff(particles_);
    const double threshold =
      params_.resample_threshold_ratio * static_cast<double>(particles_.size());
    if (n_eff >= threshold) {
      should_resample = false;
    }
  }

  if (should_resample) {
    resample();
  }
}

double MHMCLEngine::apply_metropolis_hastings(
  const Pose2D & prev_odom,
  const Pose2D & curr_odom,
  const SensorModel::PreparedScan & prepared_scan)
{
  const size_t n_particles = particles_.size();
  std::vector<double> log_obs(n_particles, -std::numeric_limits<double>::infinity());
  std::vector<double> log_joint(n_particles, -std::numeric_limits<double>::infinity());
  std::uniform_real_distribution<double> dist_u(0.0, 1.0);
  std::uniform_real_distribution<double> dist_yaw(-M_PI, M_PI);
  std::normal_distribution<double> dist_rw_xy(0.0, 0.03);
  std::normal_distribution<double> dist_rw_theta(0.0, 0.04);

  double prior_sum = 0.0;
  for (const auto & p : particles_) {
    prior_sum += std::max(0.0, p.weight);
  }
  const bool use_uniform_prior = !std::isfinite(prior_sum) || prior_sum <= 0.0;
  if (use_uniform_prior) {
    prior_sum = static_cast<double>(n_particles);
  }

  const double num_beams = static_cast<double>(prepared_scan.beams.empty() ? 1 : prepared_scan.beams.size());
  constexpr double kEffectiveBeams = 2.5;
  const double beam_scale = std::min(1.0, kEffectiveBeams / num_beams);
  const auto & sp = sensor_model_.get_parameters();
  const double max_beam_prob = std::max(
    1e-6, sp.z_hit * prepared_scan.hit_gaussian_factor + prepared_scan.random_density);

  auto compute_alpha = [&](double log_w_cand, double log_w_curr) -> double {
    if (!std::isfinite(log_w_cand)) {
      return 0.0;
    }
    if (!std::isfinite(log_w_curr)) {
      return 1.0;
    }
    const double score_diff = (log_w_cand - log_w_curr) / num_beams;
    return score_diff >= 0.0 ? 1.0 : std::exp(score_diff);
  };

  auto sample_valid_motion = [&](const Particle & base) -> Particle {
    Particle cand = base;
    for (int attempt = 0; attempt < 10; ++attempt) {
      cand = base;
      motion_model_.predict(cand, prev_odom, curr_odom, rng_);
      if (map_model_.is_free_position(cand.x, cand.y) &&
        map_model_.get_distance_to_obstacle(cand.x, cand.y) > 0.09)
      {
        return cand;
      }
    }
    return cand;
  };

  // Algorithm 3, lines 4-15: Metropolis-Hastings candidate evaluation at time t
  for (size_t i = 0; i < n_particles; ++i) {
    const Particle prev_particle = particles_[i];
    const double prior_weight = use_uniform_prior ?
      1.0 : std::max(1e-12, prev_particle.weight) / prior_sum;

    // Baseline state at time t: propagate particle from t-1 to t using motion model
    // so wrong-room particles cannot stay frozen at t-1 when the robot moves.
    Particle chosen = sample_valid_motion(prev_particle);
    double log_w_chosen = sensor_model_.log_likelihood(chosen, prepared_scan, map_model_);

    // MH Step 1: Propose alternative motion samples from p(x_t | x_{t-1}, u_t)
    // and accept/reject via Metropolis-Hastings ratio alpha = min(1, w_prop / w_curr)
    for (int m_step = 0; m_step < 2; ++m_step) {
      Particle proposed = sample_valid_motion(prev_particle);
      const double log_w_prop = sensor_model_.log_likelihood(proposed, prepared_scan, map_model_);
      if (dist_u(rng_) < compute_alpha(log_w_prop, log_w_chosen)) {
        chosen = proposed;
        log_w_chosen = log_w_prop;
      }
    }

    // MH Step 2: Local MCMC random-walk refinement (aligned with mh_random_walk in Villela et al.)
    if (std::isfinite(log_w_chosen)) {
      Particle rw_cand = chosen;
      rw_cand.x += dist_rw_xy(rng_);
      rw_cand.y += dist_rw_xy(rng_);
      rw_cand.theta = normalize_angle(rw_cand.theta + dist_rw_theta(rng_));
      const double log_w_rw = sensor_model_.log_likelihood(rw_cand, prepared_scan, map_model_);
      if (std::isfinite(log_w_rw) && log_w_rw > log_w_chosen) {
        chosen = rw_cand;
        log_w_chosen = log_w_rw;
      }
    } else if (dist_u(rng_) < 0.03) {
      // MH boundary proposal: if a particle is trapped inside an obstacle/wall (w_curr = 0),
      // propose a uniform free-space state with probability 0.03 (accepted with alpha = 1 if valid).
      double rx = 0.0, ry = 0.0;
      if (map_model_.sample_free_space(rx, ry, rng_)) {
        Particle global_cand(rx, ry, dist_yaw(rng_), prev_particle.weight);
        const double log_w_global = sensor_model_.log_likelihood(global_cand, prepared_scan, map_model_);
        if (std::isfinite(log_w_global)) {
          chosen = global_cand;
          log_w_chosen = log_w_global;
        }
      }
    }

    particles_[i] = chosen;
    log_obs[i] = log_w_chosen;
    if (std::isfinite(log_w_chosen)) {
      log_joint[i] = std::log(prior_weight) + log_w_chosen * beam_scale;
    }
  }

  // Line 16: Normalize particle weights using log-sum-exp
  double max_log = -std::numeric_limits<double>::infinity();
  for (const double lj : log_joint) {
    max_log = std::max(max_log, lj);
  }

  double average_likelihood = 0.0;
  if (std::isfinite(max_log)) {
    double sum_exp = 0.0;
    for (const double lj : log_joint) {
      if (std::isfinite(lj)) {
        sum_exp += std::exp(lj - max_log);
      }
    }

    const double log_normalizer = max_log + std::log(sum_exp);
    for (size_t i = 0; i < n_particles; ++i) {
      particles_[i].weight = std::isfinite(log_joint[i]) ?
        std::exp(log_joint[i] - log_normalizer) : 0.0;
    }

    // Return the average normalized per-beam observation likelihood across particles in [0, 1]
    double sum_per_beam = 0.0;
    for (const double lw : log_obs) {
      if (std::isfinite(lw)) {
        sum_per_beam += std::exp(lw / num_beams) / max_beam_prob;
      }
    }
    average_likelihood = sum_per_beam / static_cast<double>(n_particles);
  } else {
    const double uniform_weight = 1.0 / static_cast<double>(n_particles);
    for (auto & p : particles_) {
      p.weight = uniform_weight;
    }
  }

  return average_likelihood;
}

}  // namespace montecarlo_localization
