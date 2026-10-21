#include "montecarlo_localization/mhamcl_engine.hpp"

#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

namespace montecarlo_localization
{

MHAMCLEngine::MHAMCLEngine()
: MHMCLEngine(AMCLParameters{}),
  amcl_params_{}
{
}

MHAMCLEngine::MHAMCLEngine(const AMCLParameters & params)
: MHMCLEngine(params),
  amcl_params_(params)
{
}

void MHAMCLEngine::set_amcl_parameters(const AMCLParameters & params)
{
  amcl_params_ = params;
  MHMCLEngine::set_parameters(params);
}

void MHAMCLEngine::init_gaussian(
  double mean_x, double mean_y, double mean_yaw,
  double std_x, double std_y, double std_yaw)
{
  particles_.clear();
  particles_.reserve(static_cast<size_t>(amcl_params_.min_particles));

  std::normal_distribution<double> dist_x(mean_x, std::max(1e-4, std_x));
  std::normal_distribution<double> dist_y(mean_y, std::max(1e-4, std_y));
  std::normal_distribution<double> dist_yaw(mean_yaw, std::max(1e-4, std_yaw));

  const double initial_weight = 1.0 / static_cast<double>(amcl_params_.min_particles);
  for (int i = 0; i < amcl_params_.min_particles; ++i) {
    particles_.emplace_back(
      dist_x(rng_), dist_y(rng_), normalize_angle(dist_yaw(rng_)), initial_weight);
  }

  resampler_.reset_recovery();
  is_initialized_ = true;
}

bool MHAMCLEngine::init_global()
{
  if (!map_model_.is_loaded()) {
    return false;
  }

  particles_.clear();
  particles_.reserve(static_cast<size_t>(amcl_params_.max_particles));

  std::uniform_real_distribution<double> dist_yaw(-M_PI, M_PI);
  const double initial_weight = 1.0 / static_cast<double>(amcl_params_.max_particles);
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

void MHAMCLEngine::resample()
{
  if (!is_initialized_ || particles_.empty()) {
    return;
  }

  const size_t old_size = particles_.size();
  std::vector<double> cdf(old_size, 0.0);
  double cumsum = 0.0;
  for (size_t i = 0; i < old_size; ++i) {
    cumsum += particles_[i].weight;
    cdf[i] = cumsum;
  }

  if (cumsum <= 1e-12) {
    const double inv_n = 1.0 / static_cast<double>(old_size);
    for (size_t i = 0; i < old_size; ++i) {
      cdf[i] = static_cast<double>(i + 1) * inv_n;
    }
  } else {
    for (size_t i = 0; i < old_size; ++i) {
      cdf[i] /= cumsum;
    }
  }
  cdf.back() = 1.0;

  // Algorithm 3, Line 18: Random sampling rate rho = max(0, 1 - w_fast / w_slow)
  const double p_rand = resampler_.compute_recovery_probability();

  std::uniform_real_distribution<double> dist_u(0.0, 1.0);
  std::uniform_real_distribution<double> dist_yaw(-M_PI, M_PI);

  std::vector<Particle> new_particles;
  new_particles.reserve(static_cast<size_t>(amcl_params_.max_particles));
  std::unordered_set<SpatialBinKey, SpatialBinHash> visited_bins;

  int k = 0;
  double m_chi = static_cast<double>(amcl_params_.max_particles);

  const double res_x = std::max(0.01, amcl_params_.spatial_resolution_x);
  const double res_y = std::max(0.01, amcl_params_.spatial_resolution_y);
  const double res_theta = std::max(0.01, amcl_params_.spatial_resolution_theta);

  // Mixed Resampling with KLD bounds (Algorithm 3, lines 18-22)
  while (static_cast<int>(new_particles.size()) < amcl_params_.max_particles) {
    Particle p;

    if (amcl_params_.resample_params.enable_recovery && dist_u(rng_) < p_rand && map_model_.is_loaded()) {
      double rx = 0.0, ry = 0.0;
      if (map_model_.sample_free_space(rx, ry, rng_)) {
        p = Particle(rx, ry, dist_yaw(rng_), 1.0);
      } else {
        const double r = dist_u(rng_);
        auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
        const size_t idx = std::min(static_cast<size_t>(std::distance(cdf.begin(), it)), old_size - 1);
        p = particles_[idx];
      }
    } else {
      const double r = dist_u(rng_);
      auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
      const size_t idx = std::min(static_cast<size_t>(std::distance(cdf.begin(), it)), old_size - 1);
      p = particles_[idx];
    }

    new_particles.push_back(p);

    // KLD spatial binning
    SpatialBinKey key{
      static_cast<int>(std::floor(p.x / res_x)),
      static_cast<int>(std::floor(p.y / res_y)),
      static_cast<int>(std::floor(normalize_angle(p.theta) / res_theta))
    };

    if (visited_bins.insert(key).second) {
      k = static_cast<int>(visited_bins.size());
      if (k > 1) {
        const double km1 = static_cast<double>(k - 1);
        const double two_over_9km1 = 2.0 / (9.0 * km1);
        const double term = 1.0 - two_over_9km1 + std::sqrt(two_over_9km1) * amcl_params_.kld_z;
        if (term > 0.0) {
          m_chi = (km1 / (2.0 * amcl_params_.kld_err)) * (term * term * term);
        }
      }
    }

    const int current_n = static_cast<int>(new_particles.size());
    if (current_n >= amcl_params_.min_particles && (k <= 1 || current_n >= m_chi)) {
      break;
    }
  }

  // Normalize weights
  const double inv_new_m = 1.0 / static_cast<double>(new_particles.size());
  for (auto & np : new_particles) {
    np.weight = inv_new_m;
  }

  particles_ = std::move(new_particles);
}

}  // namespace montecarlo_localization
