#include "montecarlo_localization/amcl_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace montecarlo_localization
{

namespace
{

std::vector<double> make_cdf(const std::vector<Particle> & particles)
{
  std::vector<double> cdf(particles.size(), 0.0);
  double total_weight = 0.0;
  for (const auto & particle : particles) {
    total_weight += std::max(0.0, particle.weight);
  }

  if (!std::isfinite(total_weight) || total_weight <= 0.0) {
    const double uniform_weight = 1.0 / static_cast<double>(particles.size());
    for (size_t i = 0; i < particles.size(); ++i) {
      cdf[i] = static_cast<double>(i + 1) * uniform_weight;
    }
  } else {
    double cumulative_weight = 0.0;
    for (size_t i = 0; i < particles.size(); ++i) {
      cumulative_weight += std::max(0.0, particles[i].weight) / total_weight;
      cdf[i] = cumulative_weight;
    }
  }
  cdf.back() = 1.0;
  return cdf;
}

size_t draw_index(const std::vector<double> & cdf, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  const double value = distribution(rng);
  const auto it = std::upper_bound(cdf.begin(), cdf.end(), value);
  return std::min(
    static_cast<size_t>(std::distance(cdf.begin(), it)), cdf.size() - 1);
}

SpatialBinKey make_bin(
  const Particle & particle,
  double resolution_x,
  double resolution_y,
  double theta_resolution)
{
  const int theta_bin_count = std::max(
    1, static_cast<int>(std::round(2.0 * M_PI / theta_resolution)));
  const double circular_theta_resolution = 2.0 * M_PI / theta_bin_count;
  double positive_theta = std::fmod(
    normalize_angle(particle.theta) + M_PI + 0.5 * circular_theta_resolution,
    2.0 * M_PI);
  if (positive_theta < 0.0) {
    positive_theta += 2.0 * M_PI;
  }
  const int theta_bin = std::min(
    theta_bin_count - 1,
    static_cast<int>(std::floor(positive_theta / circular_theta_resolution)));
  return {
    static_cast<int>(std::floor(particle.x / resolution_x)),
    static_cast<int>(std::floor(particle.y / resolution_y)),
    theta_bin};
}

double kld_particle_limit(int occupied_bins, const AMCLParameters & params)
{
  if (occupied_bins <= 1) {
    return 0.0;
  }

  const double degrees_of_freedom = static_cast<double>(occupied_bins - 1);
  const double correction = 2.0 / (9.0 * degrees_of_freedom);
  const double term =
    1.0 - correction + params.kld_z * std::sqrt(correction);
  if (term <= 0.0) {
    return static_cast<double>(params.max_particles);
  }
  return (degrees_of_freedom / (2.0 * params.kld_err)) *
         term * term * term;
}

}  // namespace

AMCLEngine::AMCLEngine()
: AMCLEngine(AMCLParameters{})
{
}

AMCLEngine::AMCLEngine(const AMCLParameters & params)
: MCLEngine(params),
  amcl_params_{}
{
  set_amcl_parameters(params);
}

void AMCLEngine::set_amcl_parameters(const AMCLParameters & params)
{
  if (params.min_particles <= 0 || params.max_particles < params.min_particles ||
    !std::isfinite(params.kld_err) || params.kld_err <= 0.0 ||
    !std::isfinite(params.kld_z) || params.kld_z <= 0.0 ||
    !std::isfinite(params.spatial_resolution_x) || params.spatial_resolution_x <= 0.0 ||
    !std::isfinite(params.spatial_resolution_y) || params.spatial_resolution_y <= 0.0 ||
    !std::isfinite(params.spatial_resolution_theta) ||
    params.spatial_resolution_theta <= 0.0 ||
    params.resample_params.alpha_slow > params.resample_params.alpha_fast)
  {
    throw std::invalid_argument("Invalid AMCL particle or KLD-Sampling parameters");
  }

  AMCLParameters normalized_params = params;
  normalized_params.num_particles = normalized_params.max_particles;
  MCLEngine::set_parameters(normalized_params);
  amcl_params_ = normalized_params;
}

void AMCLEngine::set_parameters(const MCLParameters & params)
{
  AMCLParameters adaptive_params = amcl_params_;
  static_cast<MCLParameters &>(adaptive_params) = params;
  set_amcl_parameters(adaptive_params);
}

void AMCLEngine::init_gaussian(
  double mean_x, double mean_y, double mean_yaw,
  double std_x, double std_y, double std_yaw)
{
  particles_.clear();
  particles_.reserve(static_cast<size_t>(amcl_params_.min_particles));

  std::normal_distribution<double> dist_x(mean_x, std::max(1e-4, std_x));
  std::normal_distribution<double> dist_y(mean_y, std::max(1e-4, std_y));
  std::normal_distribution<double> dist_yaw(mean_yaw, std::max(1e-4, std_yaw));

  const double initial_weight = 1.0 / amcl_params_.min_particles;
  for (int i = 0; i < amcl_params_.min_particles; ++i) {
    particles_.emplace_back(
      dist_x(rng_), dist_y(rng_), normalize_angle(dist_yaw(rng_)), initial_weight);
  }

  resampler_.reset_recovery();
  is_initialized_ = true;
}

bool AMCLEngine::init_global()
{
  if (!map_model_.is_loaded()) {
    return false;
  }

  particles_.clear();
  particles_.reserve(static_cast<size_t>(amcl_params_.max_particles));

  std::uniform_real_distribution<double> dist_yaw(-M_PI, M_PI);
  const double initial_weight = 1.0 / amcl_params_.max_particles;
  for (int i = 0; i < amcl_params_.max_particles; ++i) {
    double x = 0.0;
    double y = 0.0;
    if (!map_model_.sample_free_space(x, y, rng_)) {
      particles_.clear();
      is_initialized_ = false;
      return false;
    }
    particles_.emplace_back(x, y, dist_yaw(rng_), initial_weight);
  }

  resampler_.reset_recovery();
  is_initialized_ = true;
  return true;
}

void AMCLEngine::process_update(
  const Pose2D & prev_odom,
  const Pose2D & curr_odom,
  const sensor_msgs::msg::LaserScan & scan)
{
  if (!is_initialized_ || particles_.empty() || !map_model_.is_loaded()) {
    return;
  }

  const SensorModel::PreparedScan prepared_scan = sensor_model_.prepare_scan(scan);
  const std::vector<double> parent_cdf = make_cdf(particles_);
  const double resolution_x = amcl_params_.spatial_resolution_x;
  const double resolution_y = amcl_params_.spatial_resolution_y;
  const double resolution_theta = amcl_params_.spatial_resolution_theta;

  std::vector<Particle> generated_particles;
  std::vector<double> log_likelihoods;
  generated_particles.reserve(static_cast<size_t>(amcl_params_.max_particles));
  log_likelihoods.reserve(static_cast<size_t>(amcl_params_.max_particles));
  std::unordered_set<SpatialBinKey, SpatialBinHash> occupied_bins;
  double required_count = static_cast<double>(amcl_params_.max_particles);

  while (static_cast<int>(generated_particles.size()) < amcl_params_.max_particles) {
    Particle sample = particles_[draw_index(parent_cdf, rng_)];
    motion_model_.predict(sample, prev_odom, curr_odom, rng_);
    const double log_likelihood =
      sensor_model_.log_likelihood(sample, prepared_scan, map_model_);
    generated_particles.push_back(sample);
    log_likelihoods.push_back(log_likelihood);

    occupied_bins.insert(
      make_bin(sample, resolution_x, resolution_y, resolution_theta));
    const int occupied_bin_count = static_cast<int>(occupied_bins.size());
    if (occupied_bin_count > 1) {
      required_count = kld_particle_limit(occupied_bin_count, amcl_params_);
    }

    const int generated_count = static_cast<int>(generated_particles.size());
    if (generated_count >= amcl_params_.min_particles &&
      (occupied_bin_count <= 1 || generated_count >= required_count))
    {
      break;
    }
  }

  double max_log_likelihood = -std::numeric_limits<double>::infinity();
  for (const double log_likelihood : log_likelihoods) {
    max_log_likelihood = std::max(max_log_likelihood, log_likelihood);
  }

  double average_likelihood = 0.0;
  if (std::isfinite(max_log_likelihood)) {
    double scaled_likelihood_sum = 0.0;
    for (size_t i = 0; i < generated_particles.size(); ++i) {
      if (std::isfinite(log_likelihoods[i])) {
        scaled_likelihood_sum += std::exp(log_likelihoods[i] - max_log_likelihood);
      }
    }

    const double log_average_likelihood =
      max_log_likelihood + std::log(scaled_likelihood_sum) -
      std::log(static_cast<double>(generated_particles.size()));
    if (log_average_likelihood >= std::log(std::numeric_limits<double>::min())) {
      average_likelihood = std::exp(
        std::min(
          log_average_likelihood, std::log(std::numeric_limits<double>::max())));
    }

    const double log_normalizer = max_log_likelihood + std::log(scaled_likelihood_sum);
    for (size_t i = 0; i < generated_particles.size(); ++i) {
      generated_particles[i].weight = std::isfinite(log_likelihoods[i]) ?
        std::exp(log_likelihoods[i] - log_normalizer) : 0.0;
    }
  } else {
    const double uniform_weight = 1.0 / generated_particles.size();
    for (auto & particle : generated_particles) {
      particle.weight = uniform_weight;
    }
  }

  resampler_.update_recovery_weights(average_likelihood);
  const bool recovery_needed =
    amcl_params_.resample_params.enable_recovery &&
    resampler_.compute_recovery_probability() > 0.0;
  if (params_.resample_on_update || recovery_needed) {
    std::vector<Particle> resampled_particles;
    resampler_.resample(
      generated_particles, resampled_particles, map_model_, rng_);
    particles_ = std::move(resampled_particles);
  } else {
    particles_ = std::move(generated_particles);
  }
}

void AMCLEngine::update_sensor(const sensor_msgs::msg::LaserScan & scan)
{
  process_update(Pose2D{}, Pose2D{}, scan);
}

void AMCLEngine::resample()
{
  if (!is_initialized_ || particles_.empty()) {
    return;
  }

  const std::vector<double> cdf = make_cdf(particles_);
  const double resolution_x = amcl_params_.spatial_resolution_x;
  const double resolution_y = amcl_params_.spatial_resolution_y;
  const double resolution_theta = amcl_params_.spatial_resolution_theta;

  std::vector<Particle> resampled_particles;
  resampled_particles.reserve(static_cast<size_t>(amcl_params_.max_particles));
  std::unordered_set<SpatialBinKey, SpatialBinHash> occupied_bins;
  double required_count = static_cast<double>(amcl_params_.max_particles);

  while (static_cast<int>(resampled_particles.size()) < amcl_params_.max_particles) {
    Particle sample = particles_[draw_index(cdf, rng_)];
    resampled_particles.push_back(sample);
    occupied_bins.insert(make_bin(sample, resolution_x, resolution_y, resolution_theta));

    const int occupied_bin_count = static_cast<int>(occupied_bins.size());
    if (occupied_bin_count > 1) {
      required_count = kld_particle_limit(occupied_bin_count, amcl_params_);
    }
    const int generated_count = static_cast<int>(resampled_particles.size());
    if (generated_count >= amcl_params_.min_particles &&
      (occupied_bin_count <= 1 || generated_count >= required_count))
    {
      break;
    }
  }

  const double uniform_weight = 1.0 / resampled_particles.size();
  for (auto & particle : resampled_particles) {
    particle.weight = uniform_weight;
  }
  if (amcl_params_.resample_params.enable_recovery) {
    std::vector<Particle> augmented_particles;
    resampler_.resample(
      resampled_particles, augmented_particles, map_model_, rng_);
    resampled_particles = std::move(augmented_particles);
  }
  particles_ = std::move(resampled_particles);
}

}  // namespace montecarlo_localization
