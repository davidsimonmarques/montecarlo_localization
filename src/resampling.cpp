#include "montecarlo_localization/resampling.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace montecarlo_localization
{

Resampler::Resampler()
: params_{}
{
}

Resampler::Resampler(const ResamplingParameters & params)
: params_{}
{
  set_parameters(params);
}

void Resampler::set_parameters(const ResamplingParameters & params)
{
  if (!std::isfinite(params.alpha_slow) || !std::isfinite(params.alpha_fast) ||
    params.alpha_slow < 0.0 || params.alpha_slow > 1.0 ||
    params.alpha_fast < 0.0 || params.alpha_fast > 1.0 ||
    params.alpha_slow > params.alpha_fast ||
    !std::isfinite(params.recovery_max_prob) ||
    params.recovery_max_prob < 0.0 || params.recovery_max_prob > 1.0)
  {
    throw std::invalid_argument("Invalid Augmented MCL recovery parameters");
  }
  params_ = params;
}

double Resampler::compute_neff(const std::vector<Particle> & particles) const
{
  if (particles.empty()) {
    return 0.0;
  }
  double sum_sq = 0.0;
  for (const auto & p : particles) {
    sum_sq += p.weight * p.weight;
  }
  if (sum_sq <= 1e-12) {
    return static_cast<double>(particles.size());
  }
  return 1.0 / sum_sq;
}

void Resampler::update_recovery_weights(double avg_likelihood)
{
  if (!std::isfinite(avg_likelihood) || avg_likelihood < 0.0) {
    avg_likelihood = 0.0;
  }
  if (!initialized_) {
    w_slow_ = avg_likelihood > 0.0 ? avg_likelihood : 1.0;
    w_fast_ = avg_likelihood;
    initialized_ = true;
    return;
  }

  w_slow_ += params_.alpha_slow * (avg_likelihood - w_slow_);
  w_fast_ += params_.alpha_fast * (avg_likelihood - w_fast_);
}

void Resampler::reset_recovery()
{
  initialized_ = false;
  w_slow_ = 0.0;
  w_fast_ = 0.0;
}

void Resampler::resample(
  const std::vector<Particle> & in_particles,
  std::vector<Particle> & out_particles,
  const MapModel & map_model,
  std::mt19937 & rng)
{
  size_t m_count = in_particles.size();
  if (m_count == 0) {
    return;
  }

  out_particles.clear();
  out_particles.resize(m_count);

  double total_weight = 0.0;
  for (const auto & particle : in_particles) {
    total_weight += std::max(0.0, particle.weight);
  }
  const bool use_uniform_weights = !std::isfinite(total_weight) || total_weight <= 0.0;
  if (use_uniform_weights) {
    total_weight = static_cast<double>(m_count);
  }
  const double p_rand = compute_recovery_probability();

  // Low-variance sampling algorithm
  std::uniform_real_distribution<double> dist_u(0.0, 1.0 / static_cast<double>(m_count));
  std::uniform_real_distribution<double> dist_prob(0.0, 1.0);
  std::uniform_real_distribution<double> dist_yaw(-M_PI, M_PI);

  double r = dist_u(rng);
  double c = use_uniform_weights ?
    1.0 / static_cast<double>(m_count) :
    std::max(0.0, in_particles[0].weight) / total_weight;
  size_t i = 0;
  double inv_m = 1.0 / static_cast<double>(m_count);

  for (size_t m = 0; m < m_count; ++m) {
    // Check for random particle injection (Augmented MCL)
    if (params_.enable_recovery && dist_prob(rng) < p_rand && map_model.is_loaded()) {
      double rx, ry;
      if (map_model.sample_free_space(rx, ry, rng)) {
        out_particles[m] = Particle(rx, ry, dist_yaw(rng), inv_m);
        continue;
      }
    }

    double u = r + static_cast<double>(m) * inv_m;
    while (u >= c && i < m_count - 1) {
      ++i;
      c += use_uniform_weights ?
        1.0 / static_cast<double>(m_count) :
        std::max(0.0, in_particles[i].weight) / total_weight;
    }

    out_particles[m] = in_particles[i];
    out_particles[m].weight = inv_m;

  }
}

}  // namespace montecarlo_localization
