#include "montecarlo_localization/motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace montecarlo_localization
{

namespace
{

struct OdometryDelta
{
  double rot1;
  double translation;
  double rot2;
  double std_rot1;
  double std_translation;
  double std_rot2;
};

OdometryDelta make_odometry_delta(
  const MotionParameters & params,
  const Pose2D & prev_odom,
  const Pose2D & curr_odom)
{
  const double dx = curr_odom.x - prev_odom.x;
  const double dy = curr_odom.y - prev_odom.y;
  const double translation = std::hypot(dx, dy);
  const double delta_theta = angle_diff(curr_odom.theta, prev_odom.theta);

  double rot1 = 0.0;
  double rot2 = 0.0;
  if (translation > 1e-9) {
    rot1 = angle_diff(std::atan2(dy, dx), prev_odom.theta);
    rot2 = angle_diff(delta_theta, rot1);
  } else {
    rot2 = delta_theta;
  }

  const double var_rot1 =
    params.alpha1 * rot1 * rot1 + params.alpha2 * translation * translation;
  const double var_translation =
    params.alpha3 * translation * translation +
    params.alpha4 * (rot1 * rot1 + rot2 * rot2);
  const double var_rot2 =
    params.alpha1 * rot2 * rot2 + params.alpha2 * translation * translation;

  return {
    rot1,
    translation,
    rot2,
    std::sqrt(var_rot1),
    std::sqrt(var_translation),
    std::sqrt(var_rot2)};
}

void apply_odometry_sample(
  Particle & particle,
  const OdometryDelta & delta,
  std::normal_distribution<double> & dist_rot1,
  std::normal_distribution<double> & dist_translation,
  std::normal_distribution<double> & dist_rot2,
  std::mt19937 & rng)
{
  const double sampled_rot1 = delta.rot1 -
    (delta.std_rot1 > 0.0 ? dist_rot1(rng) : 0.0);
  const double sampled_translation = delta.translation -
    (delta.std_translation > 0.0 ? dist_translation(rng) : 0.0);
  const double sampled_rot2 = delta.rot2 -
    (delta.std_rot2 > 0.0 ? dist_rot2(rng) : 0.0);

  particle.x += sampled_translation * std::cos(particle.theta + sampled_rot1);
  particle.y += sampled_translation * std::sin(particle.theta + sampled_rot1);
  particle.theta = normalize_angle(particle.theta + sampled_rot1 + sampled_rot2);
}

}  // namespace

MotionModel::MotionModel()
: params_{}
{
}

MotionModel::MotionModel(const MotionParameters & params)
: params_{}
{
  set_parameters(params);
}

void MotionModel::set_parameters(const MotionParameters & params)
{
  const double alphas[] = {params.alpha1, params.alpha2, params.alpha3, params.alpha4};
  for (const double alpha : alphas) {
    if (!std::isfinite(alpha) || alpha < 0.0) {
      throw std::invalid_argument("Motion model alpha parameters must be finite and non-negative");
    }
  }
  params_ = params;
}

void MotionModel::predict(
  std::vector<Particle> & particles,
  const Pose2D & prev_odom,
  const Pose2D & curr_odom,
  std::mt19937 & rng)
{
  const OdometryDelta delta = make_odometry_delta(params_, prev_odom, curr_odom);
  std::normal_distribution<double> dist_rot1(0.0, delta.std_rot1 > 0.0 ? delta.std_rot1 : 1.0);
  std::normal_distribution<double> dist_translation(
    0.0, delta.std_translation > 0.0 ? delta.std_translation : 1.0);
  std::normal_distribution<double> dist_rot2(0.0, delta.std_rot2 > 0.0 ? delta.std_rot2 : 1.0);
  for (auto & particle : particles) {
    apply_odometry_sample(
      particle, delta, dist_rot1, dist_translation, dist_rot2, rng);
  }
}

void MotionModel::predict(
  Particle & particle,
  const Pose2D & prev_odom,
  const Pose2D & curr_odom,
  std::mt19937 & rng)
{
  const OdometryDelta delta = make_odometry_delta(params_, prev_odom, curr_odom);
  std::normal_distribution<double> dist_rot1(0.0, delta.std_rot1 > 0.0 ? delta.std_rot1 : 1.0);
  std::normal_distribution<double> dist_translation(
    0.0, delta.std_translation > 0.0 ? delta.std_translation : 1.0);
  std::normal_distribution<double> dist_rot2(0.0, delta.std_rot2 > 0.0 ? delta.std_rot2 : 1.0);
  apply_odometry_sample(
    particle, delta, dist_rot1, dist_translation, dist_rot2, rng);
}

}  // namespace montecarlo_localization
