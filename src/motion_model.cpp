#include "montecarlo_localization/motion_model.hpp"
#include <cmath>

namespace montecarlo_localization
{

MotionModel::MotionModel()
: params_{}
{
}

MotionModel::MotionModel(const MotionParameters & params)
: params_(params)
{
}

void MotionModel::predict(
  std::vector<Particle> & particles,
  const Pose2D & prev_odom,
  const Pose2D & curr_odom,
  std::mt19937 & rng)
{
  double dx = curr_odom.x - prev_odom.x;
  double dy = curr_odom.y - prev_odom.y;
  double dtheta = angle_diff(curr_odom.theta, prev_odom.theta);

  double delta_trans = std::sqrt(dx * dx + dy * dy);
  double delta_rot1 = 0.0;
  double delta_rot2 = 0.0;

  if (delta_trans > 1e-4) {
    delta_rot1 = angle_diff(std::atan2(dy, dx), prev_odom.theta);
    delta_rot2 = angle_diff(curr_odom.theta, prev_odom.theta + delta_rot1);
  } else {
    // Pure rotation
    delta_rot1 = 0.0;
    delta_rot2 = dtheta;
  }

  // Variances based on alpha parameters
  double var_rot1 = params_.alpha1 * delta_rot1 * delta_rot1 +
                    params_.alpha2 * delta_trans * delta_trans;
  double var_trans = params_.alpha3 * delta_trans * delta_trans +
                     params_.alpha4 * (delta_rot1 * delta_rot1 + delta_rot2 * delta_rot2);
  double var_rot2 = params_.alpha1 * delta_rot2 * delta_rot2 +
                    params_.alpha2 * delta_trans * delta_trans;

  double std_rot1 = std::sqrt(std::max(1e-9, var_rot1));
  double std_trans = std::sqrt(std::max(1e-9, var_trans));
  double std_rot2 = std::sqrt(std::max(1e-9, var_rot2));

  std::normal_distribution<double> dist_rot1(0.0, std_rot1);
  std::normal_distribution<double> dist_trans(0.0, std_trans);
  std::normal_distribution<double> dist_rot2(0.0, std_rot2);

  for (auto & p : particles) {
    double sampled_rot1 = delta_rot1 + dist_rot1(rng);
    double sampled_trans = delta_trans + dist_trans(rng);
    double sampled_rot2 = delta_rot2 + dist_rot2(rng);

    p.x += sampled_trans * std::cos(p.theta + sampled_rot1);
    p.y += sampled_trans * std::sin(p.theta + sampled_rot1);
    p.theta = normalize_angle(p.theta + sampled_rot1 + sampled_rot2);
  }
}

}  // namespace montecarlo_localization
