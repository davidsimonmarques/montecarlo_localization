#ifndef MONTECARLO_LOCALIZATION_MOTION_MODEL_HPP_
#define MONTECARLO_LOCALIZATION_MOTION_MODEL_HPP_

#include <vector>
#include <random>
#include "montecarlo_localization/particle.hpp"

namespace montecarlo_localization
{

struct MotionParameters
{
  double alpha1{0.05};  // Rotational noise from rotation
  double alpha2{0.05};  // Rotational noise from translation
  double alpha3{0.05};  // Translational noise from translation
  double alpha4{0.05};  // Translational noise from rotation
};

class MotionModel
{
public:
  MotionModel();
  explicit MotionModel(const MotionParameters & params);
  ~MotionModel() = default;

  void set_parameters(const MotionParameters & params) { params_ = params; }
  const MotionParameters & get_parameters() const { return params_; }

  /**
   * @brief Propagates particle state according to the odometry motion model
   * @param particles List of particles to update in-place
   * @param prev_odom Odometry pose at time t-1
   * @param curr_odom Odometry pose at time t
   * @param rng Random number generator
   */
  void predict(
    std::vector<Particle> & particles,
    const Pose2D & prev_odom,
    const Pose2D & curr_odom,
    std::mt19937 & rng);

private:
  MotionParameters params_;
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_MOTION_MODEL_HPP_
