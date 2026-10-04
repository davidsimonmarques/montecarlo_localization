#include "montecarlo_localization/sensor_model.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

namespace montecarlo_localization
{

SensorModel::SensorModel()
: params_{}
{
}

SensorModel::SensorModel(const SensorParameters & params)
: params_(params)
{
}

struct ValidBeam
{
  double range;
  double angle;
};

double SensorModel::update_weights(
  std::vector<Particle> & particles,
  const sensor_msgs::msg::LaserScan & scan,
  const MapModel & map_model)
{
  if (particles.empty() || !map_model.is_loaded()) {
    return 1.0;
  }

  // Pre-filter valid beams from LaserScan to avoid re-evaluating for every particle
  std::vector<ValidBeam> valid_beams;
  valid_beams.reserve(scan.ranges.size());

  int step = std::max(1, params_.beam_skip);
  int total_beams = static_cast<int>(scan.ranges.size());
  if (params_.max_beams > 0 && total_beams > params_.max_beams) {
    step = std::max(step, (total_beams + params_.max_beams - 1) / params_.max_beams);
  }

  for (int i = 0; i < total_beams; i += step) {
    float r = scan.ranges[i];
    if (std::isnan(r) || std::isinf(r)) {
      continue;
    }
    if (r < params_.min_range || r > params_.max_range) {
      continue;
    }
    // Likelihood field only models obstacle hits. Misses (at or near range_max) must be skipped.
    if (r >= (scan.range_max - 0.05f)) {
      continue;
    }
    double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    valid_beams.push_back({static_cast<double>(r), angle});

  }

  if (valid_beams.empty()) {
    return 1.0;
  }

  double hit_gaussian_factor = 1.0 / (std::sqrt(2.0 * M_PI) * params_.sigma_hit);
  double two_sigma_sq = 2.0 * params_.sigma_hit * params_.sigma_hit;
  double p_rand = params_.z_rand / params_.max_range;

  std::vector<double> log_weights(particles.size(), 0.0);
  double max_log_w = -std::numeric_limits<double>::infinity();

  for (size_t p_idx = 0; p_idx < particles.size(); ++p_idx) {
    const auto & p = particles[p_idx];

    int pmx, pmy;
    if (!map_model.world_to_map(p.x, p.y, pmx, pmy) || !map_model.is_free(pmx, pmy)) {
      log_weights[p_idx] = -1e9;
      continue;
    }

    // Compute laser origin in world coordinates for this particle
    double cos_p = std::cos(p.theta);
    double sin_p = std::sin(p.theta);

    double laser_world_x = p.x + params_.laser_x * cos_p - params_.laser_y * sin_p;
    double laser_world_y = p.y + params_.laser_x * sin_p + params_.laser_y * cos_p;
    double laser_world_yaw = normalize_angle(p.theta + params_.laser_yaw);

    double log_w = 0.0;

    for (const auto & beam : valid_beams) {
      double beam_angle_world = laser_world_yaw + beam.angle;
      double endpoint_x = laser_world_x + beam.range * std::cos(beam_angle_world);
      double endpoint_y = laser_world_y + beam.range * std::sin(beam_angle_world);

      double d = map_model.get_distance_to_obstacle(endpoint_x, endpoint_y);
      double p_hit = hit_gaussian_factor * std::exp(-(d * d) / two_sigma_sq);
      double p_total = params_.z_hit * p_hit + p_rand;

      if (p_total > 1e-12) {
        log_w += std::log(p_total);
      } else {
        log_w += std::log(1e-12);
      }
    }

    log_weights[p_idx] = log_w;
    if (log_w > max_log_w) {
      max_log_w = log_w;
    }
  }

  // Convert log weights to normalized linear weights with log-sum-exp stabilization
  double sum_weights = 0.0;
  for (size_t i = 0; i < particles.size(); ++i) {
    double w = std::exp(log_weights[i] - max_log_w);
    particles[i].weight = w;
    sum_weights += w;
  }

  // Normalize
  if (sum_weights > 0.0) {
    for (auto & p : particles) {
      p.weight /= sum_weights;
    }
  } else {
    double uniform = 1.0 / static_cast<double>(particles.size());
    for (auto & p : particles) {
      p.weight = uniform;
    }
  }

  // Return average raw likelihood factor for augmented recovery tracking
  double avg_likelihood = sum_weights / static_cast<double>(particles.size());
  return avg_likelihood;
}

}  // namespace montecarlo_localization
