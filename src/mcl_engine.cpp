#include "montecarlo_localization/mcl_engine.hpp"

#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

namespace montecarlo_localization
{

MCLEngine::MCLEngine()
: params_{},
  motion_model_(params_.motion_params),
  sensor_model_(params_.sensor_params),
  resampler_(params_.resample_params),
  rng_(std::random_device{}())
{
}

MCLEngine::MCLEngine(const MCLParameters & params)
: params_(params),
  motion_model_(params_.motion_params),
  sensor_model_(params_.sensor_params),
  resampler_(params_.resample_params),
  rng_(std::random_device{}())
{
}

void MCLEngine::set_parameters(const MCLParameters & params)
{
  params_ = params;
  motion_model_.set_parameters(params_.motion_params);
  sensor_model_.set_parameters(params_.sensor_params);
  resampler_.set_parameters(params_.resample_params);
}

void MCLEngine::set_map(const nav_msgs::msg::OccupancyGrid & map, int occupied_thresh, int free_thresh)
{
  map_model_.set_map(map, occupied_thresh, free_thresh);
}

void MCLEngine::init_gaussian(
  double mean_x, double mean_y, double mean_yaw,
  double std_x, double std_y, double std_yaw)
{
  particles_.clear();
  particles_.reserve(params_.num_particles);

  std::normal_distribution<double> dist_x(mean_x, std::max(1e-4, std_x));
  std::normal_distribution<double> dist_y(mean_y, std::max(1e-4, std_y));
  std::normal_distribution<double> dist_yaw(mean_yaw, std::max(1e-4, std_yaw));

  double initial_weight = 1.0 / static_cast<double>(params_.num_particles);

  for (int i = 0; i < params_.num_particles; ++i) {
    double px = dist_x(rng_);
    double py = dist_y(rng_);
    double pyaw = normalize_angle(dist_yaw(rng_));
    particles_.emplace_back(px, py, pyaw, initial_weight);
  }

  resampler_.reset_recovery();
  is_initialized_ = true;
}

bool MCLEngine::init_global()
{
  if (!map_model_.is_loaded()) {
    return false;
  }

  particles_.clear();
  particles_.reserve(params_.num_particles);

  std::uniform_real_distribution<double> dist_yaw(-M_PI, M_PI);
  double initial_weight = 1.0 / static_cast<double>(params_.num_particles);

  for (int i = 0; i < params_.num_particles; ++i) {
    double px = 0.0, py = 0.0;
    if (!map_model_.sample_free_space(px, py, rng_)) {
      return false;
    }
    double pyaw = dist_yaw(rng_);
    particles_.emplace_back(px, py, pyaw, initial_weight);
  }

  resampler_.reset_recovery();
  is_initialized_ = true;
  return true;
}

void MCLEngine::predict(const Pose2D & prev_odom, const Pose2D & curr_odom)
{
  if (!is_initialized_ || particles_.empty()) {
    return;
  }
  motion_model_.predict(particles_, prev_odom, curr_odom, rng_);
}

void MCLEngine::update_sensor(const sensor_msgs::msg::LaserScan & scan)
{
  if (!is_initialized_ || particles_.empty() || !map_model_.is_loaded()) {
    return;
  }

  double avg_likelihood = sensor_model_.update_weights(particles_, scan, map_model_);
  resampler_.update_recovery_weights(avg_likelihood);

  if (params_.resample_on_update) {
    resample();
  }
}

void MCLEngine::resample()
{
  if (!is_initialized_ || particles_.empty()) {
    return;
  }
  std::vector<Particle> new_particles;
  resampler_.resample(particles_, new_particles, map_model_, rng_);
  particles_ = std::move(new_particles);
}

bool MCLEngine::has_moved(const Pose2D & last_update_odom, const Pose2D & curr_odom) const
{
  double dx = curr_odom.x - last_update_odom.x;
  double dy = curr_odom.y - last_update_odom.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  double dtheta = std::abs(angle_diff(curr_odom.theta, last_update_odom.theta));

  return (dist >= params_.update_min_d || dtheta >= params_.update_min_a);
}

PoseEstimate MCLEngine::get_estimate() const
{
  PoseEstimate est;
  est.covariance.fill(0.0);

  if (particles_.empty()) {
    return est;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  double sum_sin = 0.0;
  double sum_cos = 0.0;
  double total_weight = 0.0;

  for (const auto & p : particles_) {
    mean_x += p.weight * p.x;
    mean_y += p.weight * p.y;
    sum_sin += p.weight * std::sin(p.theta);
    sum_cos += p.weight * std::cos(p.theta);
    total_weight += p.weight;
  }

  if (total_weight <= 0.0) {
    return est;
  }

  mean_x /= total_weight;
  mean_y /= total_weight;
  double mean_yaw = std::atan2(sum_sin, sum_cos);
  est.pose.x = mean_x;
  est.pose.y = mean_y;
  est.pose.theta = mean_yaw;

  double cov_xx = 0.0;
  double cov_yy = 0.0;
  double cov_xy = 0.0;
  double cov_tt = 0.0;
  double cov_xt = 0.0;
  double cov_yt = 0.0;

  for (const auto & p : particles_) {
    double dx = p.x - mean_x;
    double dy = p.y - mean_y;
    double dt = angle_diff(p.theta, mean_yaw);

    cov_xx += p.weight * dx * dx;
    cov_yy += p.weight * dy * dy;
    cov_xy += p.weight * dx * dy;
    cov_tt += p.weight * dt * dt;
    cov_xt += p.weight * dx * dt;
    cov_yt += p.weight * dy * dt;
  }

  cov_xx /= total_weight;
  cov_yy /= total_weight;
  cov_xy /= total_weight;
  cov_tt /= total_weight;
  cov_xt /= total_weight;
  cov_yt /= total_weight;

  est.covariance[0] = cov_xx;
  est.covariance[1] = cov_xy;
  est.covariance[2] = cov_xt;
  est.covariance[3] = cov_xy;
  est.covariance[4] = cov_yy;
  est.covariance[5] = cov_yt;
  est.covariance[6] = cov_xt;
  est.covariance[7] = cov_yt;
  est.covariance[8] = cov_tt;

  return est;
}

}  // namespace montecarlo_localization
