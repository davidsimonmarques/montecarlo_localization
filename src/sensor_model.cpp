#include "montecarlo_localization/sensor_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace montecarlo_localization
{

SensorModel::SensorModel()
: params_{}
{
}

SensorModel::SensorModel(const SensorParameters & params)
: params_{}
{
  set_parameters(params);
}

void SensorModel::set_parameters(const SensorParameters & params)
{
  if (!std::isfinite(params.max_range) || params.max_range <= 0.0 ||
    !std::isfinite(params.min_range) || params.min_range < 0.0 ||
    params.min_range > params.max_range ||
    !std::isfinite(params.sigma_hit) || params.sigma_hit <= 0.0 ||
    !std::isfinite(params.z_hit) || !std::isfinite(params.z_rand) ||
    params.z_hit < 0.0 || params.z_rand < 0.0 ||
    std::abs(params.z_hit + params.z_rand - 1.0) > 1e-6)
  {
    throw std::invalid_argument(
            "Sensor parameters require valid ranges, positive sigma_hit, and mixture weights summing to one");
  }
  params_ = params;
}

SensorModel::PreparedScan SensorModel::prepare_scan(
  const sensor_msgs::msg::LaserScan & scan) const
{
  PreparedScan prepared;
  const int total_beams = static_cast<int>(scan.ranges.size());
  int step = std::max(1, params_.beam_skip);
  if (params_.max_beams > 0 && total_beams > params_.max_beams) {
    step = std::max(step, (total_beams + params_.max_beams - 1) / params_.max_beams);
  }

  prepared.beams.reserve(scan.ranges.size());
  for (int i = 0; i < total_beams; i += step) {
    const float range = scan.ranges[static_cast<size_t>(i)];
    if (!std::isfinite(range) || range < params_.min_range ||
      range > params_.max_range || range >= scan.range_max)
    {
      continue;
    }
    prepared.beams.push_back(
      {
        static_cast<double>(range),
        scan.angle_min + static_cast<double>(i) * scan.angle_increment});
  }

  prepared.random_density = params_.z_rand / params_.max_range;
  prepared.hit_gaussian_factor =
    1.0 / (std::sqrt(2.0 * M_PI) * params_.sigma_hit);
  prepared.two_sigma_sq = 2.0 * params_.sigma_hit * params_.sigma_hit;
  return prepared;
}

double SensorModel::log_likelihood(
  const Particle & particle,
  const PreparedScan & scan,
  const MapModel & map_model) const
{
  int map_x = 0;
  int map_y = 0;
  if (!map_model.world_to_map(particle.x, particle.y, map_x, map_y) ||
    !map_model.is_free(map_x, map_y))
  {
    return -std::numeric_limits<double>::infinity();
  }
  if (scan.beams.empty()) {
    return 0.0;
  }

  const double cos_theta = std::cos(particle.theta);
  const double sin_theta = std::sin(particle.theta);
  const double laser_x =
    particle.x + params_.laser_x * cos_theta - params_.laser_y * sin_theta;
  const double laser_y =
    particle.y + params_.laser_x * sin_theta + params_.laser_y * cos_theta;
  const double laser_theta = normalize_angle(particle.theta + params_.laser_yaw);

  double log_probability = 0.0;
  for (const auto & beam : scan.beams) {
    const double beam_theta = laser_theta + beam.angle;
    const double endpoint_x = laser_x + beam.range * std::cos(beam_theta);
    const double endpoint_y = laser_y + beam.range * std::sin(beam_theta);
    const double distance = map_model.get_distance_to_obstacle(endpoint_x, endpoint_y);
    const double p_hit = scan.hit_gaussian_factor *
      std::exp(-(distance * distance) / scan.two_sigma_sq);
    const double p_total = params_.z_hit * p_hit + scan.random_density;
    if (p_total <= 0.0) {
      return -std::numeric_limits<double>::infinity();
    }
    log_probability += std::log(p_total);
  }
  return log_probability;
}

double SensorModel::update_weights(
  std::vector<Particle> & particles,
  const sensor_msgs::msg::LaserScan & scan,
  const MapModel & map_model)
{
  return update_weights(particles, prepare_scan(scan), map_model);
}

double SensorModel::update_weights(
  std::vector<Particle> & particles,
  const PreparedScan & scan,
  const MapModel & map_model) const
{
  if (particles.empty() || !map_model.is_loaded()) {
    return 1.0;
  }

  double prior_sum = 0.0;
  for (const auto & particle : particles) {
    prior_sum += std::max(0.0, particle.weight);
  }
  const bool use_uniform_prior = !std::isfinite(prior_sum) || prior_sum <= 0.0;
  if (use_uniform_prior) {
    prior_sum = static_cast<double>(particles.size());
  }

  std::vector<double> log_joint(particles.size(), -std::numeric_limits<double>::infinity());
  double max_log_joint = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < particles.size(); ++i) {
    const double prior_weight = use_uniform_prior ?
      1.0 : std::max(0.0, particles[i].weight) / prior_sum;
    if (prior_weight <= 0.0) {
      continue;
    }

    const double observation_log_likelihood =
      log_likelihood(particles[i], scan, map_model);
    if (!std::isfinite(observation_log_likelihood)) {
      continue;
    }
    log_joint[i] = std::log(prior_weight) + observation_log_likelihood;
    max_log_joint = std::max(max_log_joint, log_joint[i]);
  }

  if (!std::isfinite(max_log_joint)) {
    for (auto & particle : particles) {
      particle.weight = use_uniform_prior ?
        1.0 / static_cast<double>(particles.size()) :
        std::max(0.0, particle.weight) / prior_sum;
    }
    return 0.0;
  }

  double scaled_evidence = 0.0;
  for (const double log_weight : log_joint) {
    if (std::isfinite(log_weight)) {
      scaled_evidence += std::exp(log_weight - max_log_joint);
    }
  }

  const double log_evidence = max_log_joint + std::log(scaled_evidence);
  for (size_t i = 0; i < particles.size(); ++i) {
    particles[i].weight = std::isfinite(log_joint[i]) ?
      std::exp(log_joint[i] - log_evidence) : 0.0;
  }

  if (log_evidence < std::log(std::numeric_limits<double>::min())) {
    return 0.0;
  }
  return std::exp(
    std::min(
      log_evidence, std::log(std::numeric_limits<double>::max())));
}

}  // namespace montecarlo_localization
