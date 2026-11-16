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

  for (size_t i = 0; i < particles_.size(); ++i) {
    const Particle & base = previous_particles_[i];
    bool valid = false;
    for (int attempt = 0; attempt < 25; ++attempt) {
      Particle candidate = base;
      motion_model_.predict(candidate, prev_odom, curr_odom, rng_);
      if (map_model_.is_free_position(candidate.x, candidate.y) &&
        map_model_.get_distance_to_obstacle(candidate.x, candidate.y) > 0.09)
      {
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
  std::vector<double> previous_weights(n_particles, 0.0);
  std::vector<double> proposed_weights(n_particles, 0.0);
  std::uniform_real_distribution<double> dist_u(0.0, 1.0);

  for (size_t i = 0; i < n_particles; ++i) {
    const Particle previous = previous_particles_[i];
    log_previous_likelihoods[i] =
      sensor_model_.log_likelihood(previous, prepared_scan, map_model_);
    log_proposed_likelihoods[i] =
      sensor_model_.log_likelihood(particles_[i], prepared_scan, map_model_);
  }

  const auto normalize_log_likelihoods = [](
    const std::vector<double> & log_likelihoods,
    std::vector<double> & weights)
    {
      const double max_log_likelihood =
        *std::max_element(log_likelihoods.begin(), log_likelihoods.end());
      if (!std::isfinite(max_log_likelihood)) {
        return false;
      }

      double weight_sum = 0.0;
      for (size_t i = 0; i < log_likelihoods.size(); ++i) {
        if (std::isfinite(log_likelihoods[i])) {
          weights[i] = std::exp(log_likelihoods[i] - max_log_likelihood);
          weight_sum += weights[i];
        }
      }
      if (weight_sum <= 0.0 || !std::isfinite(weight_sum)) {
        return false;
      }
      for (double & weight : weights) {
        weight /= weight_sum;
      }
      return true;
    };

  const bool has_previous_weights =
    normalize_log_likelihoods(log_previous_likelihoods, previous_weights);
  const bool has_proposed_weights =
    normalize_log_likelihoods(log_proposed_likelihoods, proposed_weights);
  const double num_beams =
    static_cast<double>(std::max<size_t>(1, prepared_scan.beams.size()));
  const auto & sensor_params = sensor_model_.get_parameters();
  const double max_beam_prob = std::max(
    1e-6,
    sensor_params.z_hit * prepared_scan.hit_gaussian_factor + prepared_scan.random_density);

  // Previous per-beam strategy, retained for comparison:
  // bool accept_proposal = false;
  // if (std::isfinite(log_w_proposed)) {
  //   if (!std::isfinite(log_w_previous)) {
  //     accept_proposal = true;
  //   } else {
  //     const double log_diff = (log_w_proposed - log_w_previous) / num_beams;
  //     const double log_alpha = std::min(0.0, log_diff);
  //     accept_proposal = dist_u(rng_) < std::exp(log_alpha);
  //   }
  // }

  double selected_weight_sum = 0.0;
  double likelihood_sum = 0.0;
  for (size_t i = 0; i < n_particles; ++i) {
    bool accept_proposal = false;
    const double previous_weight = has_previous_weights ? previous_weights[i] : 0.0;
    const double proposed_weight = has_proposed_weights ? proposed_weights[i] : 0.0;

    if (proposed_weight > 0.0) {
      if (previous_weight <= 0.0) {
        accept_proposal = true;
      } else {
        const double acceptance_probability =
          std::min(1.0, proposed_weight / previous_weight);
        accept_proposal = dist_u(rng_) < acceptance_probability;
      }
    }

    if (!accept_proposal) {
      particles_[i] = previous_particles_[i];
    }
    particles_[i].weight = accept_proposal ? proposed_weight : previous_weight;
    selected_weight_sum += particles_[i].weight;
    const double selected_log_likelihood = accept_proposal ?
      log_proposed_likelihoods[i] : log_previous_likelihoods[i];
    if (std::isfinite(selected_log_likelihood)) {
      likelihood_sum += std::exp(selected_log_likelihood / num_beams) / max_beam_prob;
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
