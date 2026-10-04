#include "montecarlo_localization/resampling.hpp"

#include <cmath>
#include <algorithm>

namespace montecarlo_localization
{

Resampler::Resampler()
: params_{}
{
}

Resampler::Resampler(const ResamplingParameters & params)
: params_(params)
{
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
  if (!initialized_) {
    w_slow_ = avg_likelihood;
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

  // Compute kidnapping random injection probability
  double p_rand = 0.0;
  if (params_.enable_recovery && w_slow_ > 1e-9) {
    p_rand = std::max(0.0, 1.0 - (w_fast_ / w_slow_));
    p_rand = std::min(p_rand, params_.recovery_max_prob);
  }

  // Low-variance sampling algorithm
  std::uniform_real_distribution<double> dist_u(0.0, 1.0 / static_cast<double>(m_count));
  std::uniform_real_distribution<double> dist_prob(0.0, 1.0);
  std::uniform_real_distribution<double> dist_yaw(-M_PI, M_PI);

  double r = dist_u(rng);
  double c = in_particles[0].weight;
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
    while (u > c && i < m_count - 1) {
      ++i;
      c += in_particles[i].weight;
    }

    out_particles[m] = in_particles[i];
    out_particles[m].weight = inv_m;

  }
}

}  // namespace montecarlo_localization
