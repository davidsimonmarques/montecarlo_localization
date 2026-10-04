#ifndef MONTECARLO_LOCALIZATION_RESAMPLING_HPP_
#define MONTECARLO_LOCALIZATION_RESAMPLING_HPP_

#include <vector>
#include <random>
#include "montecarlo_localization/particle.hpp"
#include "montecarlo_localization/map_model.hpp"

namespace montecarlo_localization
{

struct ResamplingParameters
{
  double alpha_slow{0.001};       // Decay rate for slow average weight filter
  double alpha_fast{0.1};         // Decay rate for fast average weight filter
  bool enable_recovery{false};    // Enable Augmented MCL random particle injection
  double recovery_max_prob{0.2};  // Cap maximum probability of random particle injection
};

class Resampler
{
public:
  Resampler();
  explicit Resampler(const ResamplingParameters & params);
  ~Resampler() = default;

  void set_parameters(const ResamplingParameters & params) { params_ = params; }
  const ResamplingParameters & get_parameters() const { return params_; }

  /**
   * @brief Compute Effective Sample Size (N_eff = 1 / sum(w^2))
   */
  double compute_neff(const std::vector<Particle> & particles) const;

  /**
   * @brief Update slow and fast weight exponential filters
   */
  void update_recovery_weights(double avg_likelihood);

  /**
   * @brief Reset recovery filters to initial condition
   */
  void reset_recovery();

  /**
   * @brief Perform Low-Variance Resampling with optional Augmented MCL recovery
   * @param in_particles Input particles
   * @param out_particles Output resampled particles
   * @param map_model Map used to sample random particles if recovery triggered
   * @param rng Random generator
   */
  void resample(
    const std::vector<Particle> & in_particles,
    std::vector<Particle> & out_particles,
    const MapModel & map_model,
    std::mt19937 & rng);

private:
  ResamplingParameters params_;
  double w_slow_{0.0};
  double w_fast_{0.0};
  bool initialized_{false};
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_RESAMPLING_HPP_
