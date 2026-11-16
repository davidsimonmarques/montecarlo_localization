#include "montecarlo_localization/fastlio_bridge_node.hpp"

#include <algorithm>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace montecarlo_localization
{

FastlioBridgeNode::FastlioBridgeNode(const rclcpp::NodeOptions & options)
: Node("fastlio_bridge_node", options)
{
  declare_and_get_parameters();
  precompute_ground_ring();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  publish_static_odom_to_fastlio_world();

  // Publishers (Reliable QoS is compatible with both Reliable and BestEffort subscribers)
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_topic_, 20);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_out_topic_, 50);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_out_topic_, 20);

  // Subscribers (SensorDataQoS receives from both Reliable and BestEffort simulator publishers)
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FastlioBridgeNode::scan_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FastlioBridgeNode::imu_callback, this, std::placeholders::_1));

  fastlio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    fastlio_odom_in_topic_, 20,
    std::bind(&FastlioBridgeNode::fastlio_odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "FAST-LIO2 bridge initialized: scan='%s' + imu='%s' -> cloud='%s' + imu='%s', "
    "FAST-LIO odom '%s' -> '%s' (publish_odom_tf=%s)",
    scan_topic_.c_str(), imu_topic_.c_str(),
    cloud_out_topic_.c_str(), imu_out_topic_.c_str(),
    fastlio_odom_in_topic_.c_str(), odom_out_topic_.c_str(),
    publish_odom_tf_ ? "true" : "false");
}

void FastlioBridgeNode::declare_and_get_parameters()
{
  this->declare_parameter<std::string>("scan_topic", "/scan");
  this->declare_parameter<std::string>("imu_topic", "/imu/data");
  this->declare_parameter<std::string>("cloud_out_topic", "/fastlio2/scan_cloud");
  this->declare_parameter<std::string>("imu_out_topic", "/fastlio2/imu");
  this->declare_parameter<std::string>("fastlio_odom_in_topic", "/Odometry");
  this->declare_parameter<std::string>("odom_out_topic", "/odom");

  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  this->declare_parameter<std::string>("fastlio_world_frame_id", "camera_init");
  this->declare_parameter<bool>("publish_odom_tf", true);

  this->declare_parameter<int>("vertical_layers", 5);
  this->declare_parameter<double>("vertical_z_min", -0.30);
  this->declare_parameter<double>("vertical_z_max", 0.30);
  this->declare_parameter<bool>("enable_ground_plane", true);
  this->declare_parameter<double>("ground_z", -0.60);
  this->declare_parameter<double>("min_range", 0.15);
  this->declare_parameter<double>("max_range", 9.95);
  this->declare_parameter<double>("max_range_margin", 0.05);
  this->declare_parameter<double>("stamp_offset_sec", 0.001);

  this->declare_parameter<bool>("ensure_imu_gravity", true);
  this->declare_parameter<double>("gravity_mps2", 9.81);

  // Fallback extrinsics relative to base_footprint when TF lookup is not available
  this->declare_parameter<double>("laser_offset_x", 0.0);
  this->declare_parameter<double>("laser_offset_y", 0.0);
  this->declare_parameter<double>("laser_offset_z", 0.0);
  this->declare_parameter<double>("laser_offset_roll", 0.0);
  this->declare_parameter<double>("laser_offset_pitch", 0.0);
  this->declare_parameter<double>("laser_offset_yaw", 0.0);

  this->declare_parameter<double>("imu_offset_x", 0.0);
  this->declare_parameter<double>("imu_offset_y", 0.0);
  this->declare_parameter<double>("imu_offset_z", 0.0);
  this->declare_parameter<double>("imu_offset_roll", 0.0);
  this->declare_parameter<double>("imu_offset_pitch", 0.0);
  this->declare_parameter<double>("imu_offset_yaw", 0.0);

  scan_topic_ = this->get_parameter("scan_topic").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();
  cloud_out_topic_ = this->get_parameter("cloud_out_topic").as_string();
  imu_out_topic_ = this->get_parameter("imu_out_topic").as_string();
  fastlio_odom_in_topic_ = this->get_parameter("fastlio_odom_in_topic").as_string();
  odom_out_topic_ = this->get_parameter("odom_out_topic").as_string();

  odom_frame_id_ = strip_leading_slash(this->get_parameter("odom_frame_id").as_string());
  base_frame_id_ = strip_leading_slash(this->get_parameter("base_frame_id").as_string());
  fastlio_world_frame_id_ = strip_leading_slash(
    this->get_parameter("fastlio_world_frame_id").as_string());
  publish_odom_tf_ = this->get_parameter("publish_odom_tf").as_bool();

  vertical_layers_ = std::max(3, static_cast<int>(this->get_parameter("vertical_layers").as_int()));
  vertical_z_min_ = this->get_parameter("vertical_z_min").as_double();
  vertical_z_max_ = this->get_parameter("vertical_z_max").as_double();
  enable_ground_plane_ = this->get_parameter("enable_ground_plane").as_bool();
  ground_z_ = this->get_parameter("ground_z").as_double();
  min_range_ = this->get_parameter("min_range").as_double();
  max_range_ = this->get_parameter("max_range").as_double();
  max_range_margin_ = this->get_parameter("max_range_margin").as_double();
  stamp_offset_sec_ = this->get_parameter("stamp_offset_sec").as_double();

  ensure_imu_gravity_ = this->get_parameter("ensure_imu_gravity").as_bool();
  gravity_mps2_ = this->get_parameter("gravity_mps2").as_double();

  tf2::Quaternion q_laser;
  q_laser.setRPY(
    this->get_parameter("laser_offset_roll").as_double(),
    this->get_parameter("laser_offset_pitch").as_double(),
    this->get_parameter("laser_offset_yaw").as_double());
  t_base_laser_.setOrigin(
    tf2::Vector3(
      this->get_parameter("laser_offset_x").as_double(),
      this->get_parameter("laser_offset_y").as_double(),
      this->get_parameter("laser_offset_z").as_double()));
  t_base_laser_.setRotation(q_laser);

  tf2::Quaternion q_imu;
  q_imu.setRPY(
    this->get_parameter("imu_offset_roll").as_double(),
    this->get_parameter("imu_offset_pitch").as_double(),
    this->get_parameter("imu_offset_yaw").as_double());
  t_base_imu_.setOrigin(
    tf2::Vector3(
      this->get_parameter("imu_offset_x").as_double(),
      this->get_parameter("imu_offset_y").as_double(),
      this->get_parameter("imu_offset_z").as_double()));
  t_base_imu_.setRotation(q_imu);

  vertical_z_offsets_.resize(static_cast<size_t>(vertical_layers_));
  if (vertical_layers_ == 1) {
    vertical_z_offsets_[0] = 0.0f;
  } else {
    const double dz =
      (vertical_z_max_ - vertical_z_min_) / static_cast<double>(vertical_layers_ - 1);
    for (int i = 0; i < vertical_layers_; ++i) {
      vertical_z_offsets_[static_cast<size_t>(i)] =
        static_cast<float>(vertical_z_min_ + static_cast<double>(i) * dz);
    }
  }
}

void FastlioBridgeNode::precompute_ground_ring()
{
  ground_xy_points_.clear();
  if (!enable_ground_plane_) {
    return;
  }

  // Three concentric horizontal rings on z = ground_z_ provide normal (0, 0, 1) to constrain
  // Z, roll, and pitch in 3D FAST-LIO2 without introducing any horizontal (X, Y, yaw) Jacobian.
  const double radii[] = {0.65, 0.85, 1.05};
  const int angular_steps = 24;
  ground_xy_points_.reserve(3 * angular_steps);

  for (const double radius : radii) {
    for (int k = 0; k < angular_steps; ++k) {
      const double angle = (2.0 * M_PI * static_cast<double>(k)) /
        static_cast<double>(angular_steps);
      ground_xy_points_.emplace_back(
        static_cast<float>(radius * std::cos(angle)),
        static_cast<float>(radius * std::sin(angle)));
    }
  }
}

void FastlioBridgeNode::publish_static_odom_to_fastlio_world()
{
  if (!static_tf_broadcaster_) {
    return;
  }
  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.stamp = this->now();
  static_tf.header.frame_id = odom_frame_id_;
  static_tf.child_frame_id = fastlio_world_frame_id_;
  static_tf.transform.translation.x = t_base_imu_.getOrigin().x();
  static_tf.transform.translation.y = t_base_imu_.getOrigin().y();
  static_tf.transform.translation.z = t_base_imu_.getOrigin().z();
  const tf2::Quaternion q = t_base_imu_.getRotation();
  static_tf.transform.rotation.x = q.x();
  static_tf.transform.rotation.y = q.y();
  static_tf.transform.rotation.z = q.z();
  static_tf.transform.rotation.w = q.w();
  static_tf_broadcaster_->sendTransform(static_tf);
}

std::string FastlioBridgeNode::strip_leading_slash(const std::string & frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

void FastlioBridgeNode::try_resolve_laser_tf(const std::string & raw_frame_id)
{
  if (laser_tf_resolved_) {
    return;
  }
  const std::string laser_frame = strip_leading_slash(raw_frame_id);
  if (laser_frame.empty() || laser_frame == base_frame_id_) {
    laser_tf_resolved_ = true;
    return;
  }

  try {
    const auto tf_msg = tf_buffer_->lookupTransform(
      base_frame_id_, laser_frame, tf2::TimePointZero);
    t_base_laser_.setOrigin(
      tf2::Vector3(
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z));
    t_base_laser_.setRotation(
      tf2::Quaternion(
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w));
    laser_tf_resolved_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Resolved static TF %s -> %s: t=(%.3f, %.3f, %.3f)",
      base_frame_id_.c_str(), laser_frame.c_str(),
      t_base_laser_.getOrigin().x(),
      t_base_laser_.getOrigin().y(),
      t_base_laser_.getOrigin().z());
  } catch (const tf2::TransformException &) {
    if (++laser_tf_lookup_attempts_ == 25) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF %s -> %s not yet available; using configured laser_offset_* fallback until resolved.",
        base_frame_id_.c_str(), laser_frame.c_str());
    }
  }
}

void FastlioBridgeNode::try_resolve_imu_tf(const std::string & raw_frame_id)
{
  if (imu_tf_resolved_) {
    return;
  }
  const std::string imu_frame = strip_leading_slash(raw_frame_id);
  if (imu_frame.empty() || imu_frame == base_frame_id_) {
    imu_tf_resolved_ = true;
    return;
  }

  try {
    const auto tf_msg = tf_buffer_->lookupTransform(
      base_frame_id_, imu_frame, tf2::TimePointZero);
    t_base_imu_.setOrigin(
      tf2::Vector3(
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z));
    t_base_imu_.setRotation(
      tf2::Quaternion(
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w));
    imu_tf_resolved_ = true;
    publish_static_odom_to_fastlio_world();
    RCLCPP_INFO(
      this->get_logger(),
      "Resolved static TF %s -> %s: t=(%.3f, %.3f, %.3f)",
      base_frame_id_.c_str(), imu_frame.c_str(),
      t_base_imu_.getOrigin().x(),
      t_base_imu_.getOrigin().y(),
      t_base_imu_.getOrigin().z());
  } catch (const tf2::TransformException &) {
    if (++imu_tf_lookup_attempts_ == 50) {
      RCLCPP_INFO(
        this->get_logger(),
        "No separate TF %s -> %s found; assuming configured imu_offset_* relative to %s.",
        base_frame_id_.c_str(), imu_frame.c_str(), base_frame_id_.c_str());
    }
  }
}

void FastlioBridgeNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  try_resolve_imu_tf(msg->header.frame_id);

  sensor_msgs::msg::Imu out_msg = *msg;
  rclcpp::Time stamp = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
    this->now() : rclcpp::Time(msg->header.stamp);

  if (last_imu_stamp_.nanoseconds() > 0 && stamp <= last_imu_stamp_) {
    stamp = last_imu_stamp_ + rclcpp::Duration(0, 100000);  // +0.1 ms monotonic step
  }
  last_imu_stamp_ = stamp;
  out_msg.header.stamp = stamp;
  out_msg.header.frame_id = "body";

  if (ensure_imu_gravity_ && !gravity_checked_) {
    const double ax = out_msg.linear_acceleration.x;
    const double ay = out_msg.linear_acceleration.y;
    const double az = out_msg.linear_acceleration.z;
    const double acc_norm = std::sqrt(ax * ax + ay * ay + az * az);
    ++imu_init_samples_;
    if (acc_norm >= 5.0) {
      inject_gravity_ = false;
      gravity_checked_ = true;
    } else if (imu_init_samples_ >= 5 && acc_norm < 1.0) {
      inject_gravity_ = true;
      gravity_checked_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "Simulator IMU '%s' reports near-zero stationary acceleration (%.3f m/s^2); "
        "injecting +%.2f m/s^2 gravity reaction so FAST-LIO2 initializes cleanly.",
        imu_topic_.c_str(), acc_norm, gravity_mps2_);
    }
  }

  if (inject_gravity_) {
    tf2::Vector3 g_sensor(0.0, 0.0, gravity_mps2_);
    const double qx = out_msg.orientation.x;
    const double qy = out_msg.orientation.y;
    const double qz = out_msg.orientation.z;
    const double qw = out_msg.orientation.w;
    const double q_norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (q_norm > 0.9) {
      tf2::Quaternion q_world_imu(qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm);
      g_sensor = tf2::quatRotate(q_world_imu.inverse(), tf2::Vector3(0.0, 0.0, gravity_mps2_));
    } else {
      g_sensor = t_base_imu_.getBasis().transpose() * tf2::Vector3(0.0, 0.0, gravity_mps2_);
    }
    out_msg.linear_acceleration.x += g_sensor.x();
    out_msg.linear_acceleration.y += g_sensor.y();
    out_msg.linear_acceleration.z += g_sensor.z();
  }

  imu_pub_->publish(out_msg);
}

void FastlioBridgeNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  try_resolve_laser_tf(msg->header.frame_id);
  if (!laser_tf_resolved_ && laser_tf_lookup_attempts_ < 3) {
    return;
  }

  rclcpp::Time raw_stamp = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
    this->now() : rclcpp::Time(msg->header.stamp);
  rclcpp::Time cloud_stamp = raw_stamp + rclcpp::Duration::from_seconds(stamp_offset_sec_);
  if (last_cloud_stamp_.nanoseconds() > 0 && cloud_stamp <= last_cloud_stamp_) {
    cloud_stamp = last_cloud_stamp_ + rclcpp::Duration(0, 100000);
  }
  last_cloud_stamp_ = cloud_stamp;

  // Broadcast identity odom -> base_footprint during FAST-LIO2's brief ~0.5 s IMU init window
  // so TF consumers (RViz / MCL laser offset lookup) have a valid tree immediately.
  if (publish_odom_tf_ && !has_fastlio_odom_) {
    tf2::Transform identity_tf;
    identity_tf.setIdentity();
    broadcast_odom_to_base_tf(identity_tf, raw_stamp);
  }

  const double eff_min_range = std::max(min_range_, static_cast<double>(msg->range_min));
  double eff_max_range = max_range_;
  if (std::isfinite(msg->range_max) && msg->range_max > 0.0f) {
    eff_max_range = std::min(
      eff_max_range, static_cast<double>(msg->range_max) - max_range_margin_);
  }

  std::vector<std::pair<double, double>> base_2d_hits;
  base_2d_hits.reserve(msg->ranges.size());

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float r = msg->ranges[i];
    if (!std::isfinite(r) || r < eff_min_range || r >= eff_max_range) {
      continue;
    }
    const double angle = static_cast<double>(msg->angle_min) +
      static_cast<double>(i) * static_cast<double>(msg->angle_increment);
    const tf2::Vector3 p_laser(
      static_cast<double>(r) * std::cos(angle),
      static_cast<double>(r) * std::sin(angle),
      0.0);
    const tf2::Vector3 p_base = t_base_laser_ * p_laser;
    base_2d_hits.emplace_back(p_base.x(), p_base.y());
  }

  if (base_2d_hits.empty()) {
    return;
  }

  const size_t wall_points = base_2d_hits.size() * vertical_z_offsets_.size();
  const size_t total_points = wall_points + ground_xy_points_.size();

  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = cloud_stamp;
  cloud_msg.header.frame_id = "body";
  cloud_msg.height = 1;
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(total_points);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");

  const tf2::Transform t_imu_base = t_base_imu_.inverse();

  // 1. Extruded vertical wall points from real 2D LaserScan obstacles
  for (const auto & hit : base_2d_hits) {
    for (const float z_off : vertical_z_offsets_) {
      const tf2::Vector3 p_imu = t_imu_base *
        tf2::Vector3(hit.first, hit.second, static_cast<double>(z_off));
      *iter_x = static_cast<float>(p_imu.x());
      *iter_y = static_cast<float>(p_imu.y());
      *iter_z = static_cast<float>(p_imu.z());
      *iter_i = 100.0f;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
    }
  }

  // 2. Horizontal floor ring points to lock Z, roll, and pitch in 3D FAST-LIO2
  for (const auto & gpt : ground_xy_points_) {
    const tf2::Vector3 p_imu = t_imu_base *
      tf2::Vector3(
      static_cast<double>(gpt.first),
      static_cast<double>(gpt.second),
      ground_z_);
    *iter_x = static_cast<float>(p_imu.x());
    *iter_y = static_cast<float>(p_imu.y());
    *iter_z = static_cast<float>(p_imu.z());
    *iter_i = 10.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_i;
  }

  cloud_pub_->publish(cloud_msg);
}

void FastlioBridgeNode::fastlio_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Ignoring non-finite FAST-LIO2 odometry sample.");
    return;
  }

  has_fastlio_odom_ = true;

  tf2::Transform t_cam_body;
  t_cam_body.setOrigin(tf2::Vector3(p.x, p.y, p.z));
  tf2::Quaternion q_cam_body(q.x, q.y, q.z, q.w);
  if (q_cam_body.length2() > 1e-8) {
    q_cam_body.normalize();
  } else {
    q_cam_body.setValue(0.0, 0.0, 0.0, 1.0);
  }
  t_cam_body.setRotation(q_cam_body);

  // T_odom_base = T_base_imu * T_camera_init_body * inv(T_base_imu)
  const tf2::Transform t_odom_base = t_base_imu_ * t_cam_body * t_base_imu_.inverse();
  const tf2::Quaternion q_odom_base = t_odom_base.getRotation();

  nav_msgs::msg::Odometry odom_out;
  odom_out.header.stamp = msg->header.stamp;
  odom_out.header.frame_id = odom_frame_id_;
  odom_out.child_frame_id = base_frame_id_;

  odom_out.pose.pose.position.x = t_odom_base.getOrigin().x();
  odom_out.pose.pose.position.y = t_odom_base.getOrigin().y();
  odom_out.pose.pose.position.z = 0.0;
  odom_out.pose.pose.orientation.x = q_odom_base.x();
  odom_out.pose.pose.orientation.y = q_odom_base.y();
  odom_out.pose.pose.orientation.z = q_odom_base.z();
  odom_out.pose.pose.orientation.w = q_odom_base.w();

  odom_out.pose.covariance = msg->pose.covariance;
  for (size_t idx : {size_t{0}, size_t{7}, size_t{14}, size_t{21}, size_t{28}, size_t{35}}) {
    if (!std::isfinite(odom_out.pose.covariance[idx]) || odom_out.pose.covariance[idx] < 1e-6) {
      odom_out.pose.covariance[idx] = 1e-6;
    }
  }

  const tf2::Matrix3x3 & r_base_imu = t_base_imu_.getBasis();
  const tf2::Vector3 v_body(
    msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  const tf2::Vector3 w_body(
    msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
  const tf2::Vector3 v_base = r_base_imu * v_body;
  const tf2::Vector3 w_base = r_base_imu * w_body;

  odom_out.twist.twist.linear.x = v_base.x();
  odom_out.twist.twist.linear.y = v_base.y();
  odom_out.twist.twist.linear.z = v_base.z();
  odom_out.twist.twist.angular.x = w_base.x();
  odom_out.twist.twist.angular.y = w_base.y();
  odom_out.twist.twist.angular.z = w_base.z();

  odom_pub_->publish(odom_out);

  if (publish_odom_tf_) {
    broadcast_odom_to_base_tf(t_odom_base, rclcpp::Time(msg->header.stamp));
  }
}

void FastlioBridgeNode::broadcast_odom_to_base_tf(
  const tf2::Transform & tf_odom_base, const rclcpp::Time & stamp)
{
  if (!publish_odom_tf_ || stamp.nanoseconds() == 0) {
    return;
  }
  if (last_odom_tf_stamp_.nanoseconds() > 0 && stamp <= last_odom_tf_stamp_) {
    return;
  }
  last_odom_tf_stamp_ = stamp;

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = odom_frame_id_;
  tf_msg.child_frame_id = base_frame_id_;
  tf_msg.transform.translation.x = tf_odom_base.getOrigin().x();
  tf_msg.transform.translation.y = tf_odom_base.getOrigin().y();
  tf_msg.transform.translation.z = 0.0;

  const tf2::Quaternion q = tf_odom_base.getRotation();
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace montecarlo_localization
