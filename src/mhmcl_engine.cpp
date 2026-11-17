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

void MHMCLEngine::predict_mh_proposal(const Pose2D & prev_odom, const Pose2D & curr_odom)
{
  previous_particles_ = particles_;

  const double dx = curr_odom.x - prev_odom.x;
  const double dy = curr_odom.y - prev_odom.y;
  const double translation = std::hypot(dx, dy);
  const double delta_theta = angle_diff(curr_odom.theta, prev_odom.theta);
  double rot1 = 0.0;
  double rot2 = delta_theta;
  if (translation > 0.01) {
    rot1 = angle_diff(std::atan2(dy, dx), prev_odom.theta);
    rot2 = angle_diff(delta_theta, rot1);
  }

  const auto & motion_params = motion_model_.get_parameters();
  constexpr double noise_floor = 1e-5;
  const double std_rot1 =
    motion_params.alpha1 * std::abs(rot1) +
    motion_params.alpha2 * std::abs(translation) + noise_floor;
  const double std_translation =
    motion_params.alpha3 * std::abs(translation) +
    motion_params.alpha4 * (std::abs(rot1) + std::abs(rot2)) + noise_floor;
  const double std_rot2 =
    motion_params.alpha1 * std::abs(rot2) +
    motion_params.alpha2 * std::abs(translation) + noise_floor;

  std::normal_distribution<double> rot1_noise(0.0, std_rot1);
  std::normal_distribution<double> translation_noise(0.0, std_translation);
  std::normal_distribution<double> rot2_noise(0.0, std_rot2);

  for (size_t i = 0; i < particles_.size(); ++i) {
    const Particle & base = previous_particles_[i];
    bool valid = false;
    for (int attempt = 0; attempt < 200; ++attempt) {
      Particle candidate = base;
      const double sampled_rot1 = rot1 + rot1_noise(rng_);
      const double sampled_translation = translation + translation_noise(rng_);
      const double sampled_rot2 = rot2 + rot2_noise(rng_);

      candidate.x += sampled_translation * std::cos(base.theta + sampled_rot1);
      candidate.y += sampled_translation * std::sin(base.theta + sampled_rot1);
      candidate.theta = normalize_angle(
        base.theta + sampled_rot1 + sampled_rot2);

      if (map_model_.is_free_position(candidate.x, candidate.y)) {
        particles_[i] = candidate;
        valid = true;
        break;
      }
    }
    if (!valid) {
      particles_[i] = base;
    }
  }
}

void MHMCLEngine::process_update(
  const Pose2D & prev_odom,
  const Pose2D & curr_odom,
  const sensor_msgs::msg::LaserScan & scan)
{
  if (!is_initialized_ || particles_.empty() || !map_model_.is_loaded()) {
    return;
  }

  predict_mh_proposal(prev_odom, curr_odom);
  const SensorModel::PreparedScan prepared_scan = sensor_model_.prepare_scan(scan);
  const double avg_likelihood = apply_metropolis_hastings(prepared_scan);
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
  const SensorModel::PreparedScan & prepared_scan)
{
  const size_t n_particles = particles_.size();
  const double neg_inf = -std::numeric_limits<double>::infinity();
  std::vector<double> log_previous_likelihoods(n_particles, neg_inf);
  std::vector<double> log_proposed_likelihoods(n_particles, neg_inf);
  std::vector<double> selected_log_weights(n_particles, neg_inf);
  std::uniform_real_distribution<double> dist_u(0.0, 1.0);

  for (size_t i = 0; i < n_particles; ++i) {
    const Particle previous = previous_particles_[i];
    log_previous_likelihoods[i] =
      sensor_model_.log_likelihood(previous, prepared_scan, map_model_);
    log_proposed_likelihoods[i] =
      sensor_model_.log_likelihood(particles_[i], prepared_scan, map_model_);
  }

  const double num_beams =
    static_cast<double>(std::max<size_t>(1, prepared_scan.beams.size()));
  const auto & sensor_params = sensor_model_.get_parameters();
  const double max_beam_prob = std::max(
    1e-6,
    sensor_params.z_hit * prepared_scan.hit_gaussian_factor + prepared_scan.random_density);

  double likelihood_sum = 0.0;
  double max_selected_log_weight = neg_inf;
  for (size_t i = 0; i < n_particles; ++i) {
    const double log_w_previous = log_previous_likelihoods[i];
    const double log_w_proposed = log_proposed_likelihoods[i];
    bool accept_proposal = false;

    if (std::isfinite(log_w_proposed)) {
      if (!std::isfinite(log_w_previous)) {
        accept_proposal = true;
      } else {
        const double log_alpha = std::min(0.0, log_w_proposed - log_w_previous);
        accept_proposal = std::log(dist_u(rng_)) < log_alpha;
      }
    }

    if (!accept_proposal) {
      particles_[i] = previous_particles_[i];
    }
    const double selected_log_likelihood = accept_proposal ?
      log_w_proposed : log_w_previous;
    selected_log_weights[i] = selected_log_likelihood;
    max_selected_log_weight = std::max(max_selected_log_weight, selected_log_likelihood);

    if (std::isfinite(selected_log_likelihood)) {
      likelihood_sum += std::exp(selected_log_likelihood / num_beams) / max_beam_prob;
    }
  }

  double selected_weight_sum = 0.0;
  if (std::isfinite(max_selected_log_weight)) {
    for (size_t i = 0; i < n_particles; ++i) {
      particles_[i].weight = std::isfinite(selected_log_weights[i]) ?
        std::exp(selected_log_weights[i] - max_selected_log_weight) : 0.0;
      selected_weight_sum += particles_[i].weight;
    }
  }

  if (selected_weight_sum > 0.0 && std::isfinite(selected_weight_sum)) {
    for (auto & particle : particles_) {
      particle.weight /= selected_weight_sum;
    }
  } else {
    const double uniform_weight = 1.0 / static_cast<double>(n_particles);
    for (auto & particle : particles_) {
      particle.weight = uniform_weight;
    }
  }

  return likelihood_sum / static_cast<double>(n_particles);
}

}  // namespace montecarlo_localization
