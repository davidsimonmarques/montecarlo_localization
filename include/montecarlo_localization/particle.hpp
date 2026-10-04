#ifndef MONTECARLO_LOCALIZATION_PARTICLE_HPP_
#define MONTECARLO_LOCALIZATION_PARTICLE_HPP_

#include <cmath>

namespace montecarlo_localization
{

/**
 * @brief 2D Pose representation (x, y, theta)
 */
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};

  Pose2D() = default;
  Pose2D(double px, double py, double pt) : x(px), y(py), theta(pt) {}
};

/**
 * @brief Represents a single hypothesis in the Monte Carlo particle filter
 */
struct Particle
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double weight{1.0};

  Particle() = default;
  Particle(double px, double py, double pt, double pw = 1.0)
  : x(px), y(py), theta(pt), weight(pw) {}
};

/**
 * @brief Normalizes an angle to [-pi, pi]
 */
inline double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/**
 * @brief Computes shortest signed angle difference (target - source) in [-pi, pi]
 */
inline double angle_diff(double target, double source)
{
  return normalize_angle(target - source);
}

/**
 * @brief Convert yaw to quaternion (assuming roll=0, pitch=0)
 */
inline void yaw_to_quaternion(double yaw, double & qx, double & qy, double & qz, double & qw)
{
  qx = 0.0;
  qy = 0.0;
  qz = std::sin(yaw * 0.5);
  qw = std::cos(yaw * 0.5);
}

/**
 * @brief Extract 2D yaw angle from quaternion
 */
inline double quaternion_to_yaw(double qx, double qy, double qz, double qw)
{
  // 2 * (w * z + x * y), 1 - 2 * (y * y + z * z)
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_PARTICLE_HPP_
