#include "montecarlo_localization/mcl_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace montecarlo_localization
{

MCLNode::MCLNode(const rclcpp::NodeOptions & options)
: Node("montecarlo_localization_node", options),
  engine_{}
{
  declare_and_get_parameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  //Subscribers:
  // QoS for map subscription: transient local for latching
  rclcpp::QoS map_qos(1);
  map_qos.transient_local();
  map_qos.reliable();

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&MCLNode::map_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    std::bind(&MCLNode::odom_callback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, 10,
    std::bind(&MCLNode::scan_callback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(&MCLNode::initial_pose_callback, this, std::placeholders::_1));
  
  //Publishers:
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/mcl_pose", 10);
  //amcl_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10);
  particlecloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);

  // TF broadcasting is handled synchronously in odom_callback and scan_callback
  // using sensor message stamps to stay synchronized with the simulation clock.

  RCLCPP_INFO(this->get_logger(), "MCL Node initialized. Waiting for map on '%s'...", map_topic_.c_str());
}

void MCLNode::declare_and_get_parameters()
{
  //Frame id declarations with default values:
  this->declare_parameter<std::string>("global_frame_id", "map");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_link");

  //Topic declarations with default values:
  this->declare_parameter<std::string>("scan_topic", "/scan");
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<std::string>("odom_topic", "/odom");

  //TF broadcast declaration with default value:
  this->declare_parameter<bool>("publish_tf", true);

  //Particle filter parameters with default values:
  this->declare_parameter<int>("num_particles", 500);
  this->declare_parameter<double>("update_min_d", 0.1);
  this->declare_parameter<double>("update_min_a", 0.1);
  this->declare_parameter<bool>("resample_on_update", true);

  this->declare_parameter<double>("alpha1", 0.05);
  this->declare_parameter<double>("alpha2", 0.05);
  this->declare_parameter<double>("alpha3", 0.05);
  this->declare_parameter<double>("alpha4", 0.05);

  this->declare_parameter<double>("z_hit", 0.85);
  this->declare_parameter<double>("z_rand", 0.15);
  this->declare_parameter<double>("sigma_hit", 0.2);
  this->declare_parameter<double>("min_range", 0.1);
  this->declare_parameter<double>("max_range", 10.0);
  this->declare_parameter<int>("beam_skip", 4);
  this->declare_parameter<int>("max_beams", 60);

  this->declare_parameter<double>("alpha_slow", 0.001);
  this->declare_parameter<double>("alpha_fast", 0.1);
  this->declare_parameter<bool>("enable_recovery", false);
  this->declare_parameter<double>("recovery_max_prob", 0.2);

  this->declare_parameter<bool>("auto_initialize", true);
  this->declare_parameter<bool>("global_localization_on_start", false);
  this->declare_parameter<double>("initial_pose_x", 0.0);
  this->declare_parameter<double>("initial_pose_y", 0.0);
  this->declare_parameter<double>("initial_pose_yaw", 0.0);
  this->declare_parameter<double>("initial_cov_x", 0.25);
  this->declare_parameter<double>("initial_cov_y", 0.25);
  this->declare_parameter<double>("initial_cov_yaw", 0.068);

  global_frame_id_ = this->get_parameter("global_frame_id").as_string();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  scan_topic_ = this->get_parameter("scan_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();

  MCLParameters params;
  params.num_particles = this->get_parameter("num_particles").as_int();
  params.update_min_d = this->get_parameter("update_min_d").as_double();
  params.update_min_a = this->get_parameter("update_min_a").as_double();
  params.resample_on_update = this->get_parameter("resample_on_update").as_bool();

  params.motion_params.alpha1 = this->get_parameter("alpha1").as_double();
  params.motion_params.alpha2 = this->get_parameter("alpha2").as_double();
  params.motion_params.alpha3 = this->get_parameter("alpha3").as_double();
  params.motion_params.alpha4 = this->get_parameter("alpha4").as_double();

  params.sensor_params.z_hit = this->get_parameter("z_hit").as_double();
  params.sensor_params.z_rand = this->get_parameter("z_rand").as_double();
  params.sensor_params.sigma_hit = this->get_parameter("sigma_hit").as_double();
  params.sensor_params.min_range = this->get_parameter("min_range").as_double();
  params.sensor_params.max_range = this->get_parameter("max_range").as_double();
  params.sensor_params.beam_skip = this->get_parameter("beam_skip").as_int();
  params.sensor_params.max_beams = this->get_parameter("max_beams").as_int();

  params.resample_params.alpha_slow = this->get_parameter("alpha_slow").as_double();
  params.resample_params.alpha_fast = this->get_parameter("alpha_fast").as_double();
  params.resample_params.enable_recovery = this->get_parameter("enable_recovery").as_bool();
  params.resample_params.recovery_max_prob = this->get_parameter("recovery_max_prob").as_double();

  auto_initialize_ = this->get_parameter("auto_initialize").as_bool();
  global_localization_on_start_ = this->get_parameter("global_localization_on_start").as_bool();
  init_x_ = this->get_parameter("initial_pose_x").as_double();
  init_y_ = this->get_parameter("initial_pose_y").as_double();
  init_yaw_ = this->get_parameter("initial_pose_yaw").as_double();
  init_cov_x_ = this->get_parameter("initial_cov_x").as_double();
  init_cov_y_ = this->get_parameter("initial_cov_y").as_double();
  init_cov_yaw_ = this->get_parameter("initial_cov_yaw").as_double();

  engine_.set_parameters(params);
}

void MCLNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received map: %u x %u cells, resolution: %.4f m/cell. Computing distance field...",
    msg->info.width, msg->info.height, msg->info.resolution);

  // 1. Process occupancy grid and precompute the Euclidean Distance Transform (EDT).
  //Converts from 2D occupancy grid to continue distance field
  //Example:
  /*Occupancy Grid Map:               Grid with EDT (Distance in meters):
  [ 0 ] [ 0 ] [ 0 ] [ 100 ]            [ 0.6 ] [ 0.4 ] [ 0.2 ] [ 0.0 ]
  [ 0 ] [ 0 ] [ 0 ] [ 100 ]     -->    [ 0.6 ] [ 0.4 ] [ 0.2 ] [ 0.0 ]
  [ 0 ] [ 0 ] [ 0 ] [ 100 ]            [ 0.6 ] [ 0.4 ] [ 0.2 ] [ 0.0 ]
  (0 = free, 100 = wall)              (continuous distance to the nearest wall)
  */
  // This allows the sensor model (Likelihood Field) to query in O(1) time
  // the distance from each laser beam endpoint to the nearest obstacle.
  engine_.set_map(*msg);

  RCLCPP_INFO(this->get_logger(), "Map distance field computed successfully.");

  // 2. Particle initialization (Bootstrapping):
  // If the filter is not yet initialized, initialize particles based on configured parameters.
  if (!engine_.is_initialized()) {
    if (global_localization_on_start_) {
      // Global localization: distribute particles uniformly across free map cells only
      RCLCPP_INFO(this->get_logger(), "Initializing particles uniformly across map free space.");
      engine_.init_global();
    } else if (auto_initialize_) {
      // Auto-initialization: generate a Gaussian distribution around the initial pose configured in YAML
      RCLCPP_INFO(
        this->get_logger(),
        "Auto-initializing particles at (x=%.2f, y=%.2f, yaw=%.2f) with std=(%.2f, %.2f, %.2f).",
        init_x_, init_y_, init_yaw_,
        std::sqrt(init_cov_x_), std::sqrt(init_cov_y_), std::sqrt(init_cov_yaw_));
      engine_.init_gaussian(
        init_x_, init_y_, init_yaw_,
        std::sqrt(init_cov_x_), std::sqrt(init_cov_y_), std::sqrt(init_cov_yaw_));
    }

    // 3. Publish initial state immediately to /particlecloud and /mcl_pose,
    // allowing the map and particles to be visible in RViz before the robot starts moving.
    publish_pose_and_particles(this->now());
  }
}

void MCLNode::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  double px = msg->pose.pose.position.x;
  double py = msg->pose.pose.position.y;
  double pyaw = quaternion_to_yaw(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  //Standard deviations of the initial pose
  //std::max is used to prevent division by zero or taking the square root of a negative number
  double std_x = std::sqrt(std::max(1e-4, msg->pose.covariance[0])); 
  double std_y = std::sqrt(std::max(1e-4, msg->pose.covariance[7]));
  double std_yaw = std::sqrt(std::max(1e-4, msg->pose.covariance[35]));
  //0,7,35 are the positions of the x, y, and yaw standard deviations in the covariance matrix

  //Creates a Gaussian distribution of particles around the initial pose:
  RCLCPP_INFO(
    this->get_logger(),
    "Setting initial pose: x=%.3f, y=%.3f, yaw=%.3f (std_x=%.3f, std_y=%.3f, std_yaw=%.3f)",
    px, py, pyaw, std_x, std_y, std_yaw);

  engine_.init_gaussian(px, py, pyaw, std_x, std_y, std_yaw);
  last_update_odom_pose_ = curr_odom_pose_;
  force_initial_update_ = true;
  scan_update_count_ = 0;

  rclcpp::Time stamp = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
                        last_odom_stamp_ : rclcpp::Time(msg->header.stamp);
  publish_pose_and_particles(stamp);
  auto est = engine_.get_estimate();
  update_map_to_odom_transform(est, stamp);
}

void MCLNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_stamp_ = msg->header.stamp;
  double px = msg->pose.pose.position.x;
  double py = msg->pose.pose.position.y;
  double pyaw = quaternion_to_yaw(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  //It is necessary two odometry readings to compute the motion of the robot
  Pose2D new_odom(px, py, pyaw);
  if (!has_odom_) {
    prev_odom_pose_ = new_odom;
    curr_odom_pose_ = new_odom;
    last_update_odom_pose_ = new_odom;
    has_odom_ = true;
    return;
  }

  prev_odom_pose_ = curr_odom_pose_; //current odom becomes previous odometry for next iteration
  curr_odom_pose_ = new_odom; //new odometry is stored

  //[PREDICTION] updates particles positions based on motion model
  if (engine_.is_initialized()) { //Only updates particles if they're already initialized
    engine_.predict(prev_odom_pose_, curr_odom_pose_); //predicts the new pose of the robot
    auto est = engine_.get_estimate(); //gets the estimated pose of the robot
    update_map_to_odom_transform(est, msg->header.stamp); //updates the transform from the map frame to the odom frame
  }
}

void MCLNode::update_laser_offset_from_tf(const std::string & laser_frame)
{
  if (laser_offset_resolved_) { //To only get the laser offset once
    return;
  }
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      base_frame_id_, laser_frame, tf2::TimePointZero); //finds the transform from the base frame to the laser frame

    double lx = tf_stamped.transform.translation.x;
    double ly = tf_stamped.transform.translation.y;
    double lyaw = quaternion_to_yaw(
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w);

    engine_.sensor_model().set_laser_offset(lx, ly, lyaw); //sets the laser offset in the sensor model
    laser_offset_resolved_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Resolved laser sensor offset relative to %s: x=%.3f, y=%.3f, yaw=%.3f",
      base_frame_id_.c_str(), lx, ly, lyaw);
  } catch (const tf2::TransformException & ex) {
    // Retry next scan
  }
}

void MCLNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!laser_offset_resolved_) { //To only get the laser offset once
    update_laser_offset_from_tf(msg->header.frame_id); //finds the transform from the base frame to the laser frame
  }

  if (!engine_.is_initialized() || !has_odom_) { //Only updates particles if they're already initialized and they have moved
    return;
  }

  bool moved = engine_.has_moved(last_update_odom_pose_, curr_odom_pose_);

  // Determine if we should force an update even without movement:
  // After initialization, do one update to evaluate weights and publish initial pose/TF.
  // Subsequent updates require actual robot motion (moved == true) to prevent particle deprivation.
  bool force = false;
  if (force_initial_update_) {
    force = true;  // First update after init
  }

  if (moved || force) {
    engine_.update_sensor(*msg); //updates particles based on sensor data
    last_update_odom_pose_ = curr_odom_pose_; //current odom becomes previous odometry for next iteration
    scan_update_count_++;

    if (force_initial_update_) {
      force_initial_update_ = false;
      RCLCPP_INFO(this->get_logger(), "First sensor update completed. Publishing initial pose estimate.");
    }

    if (global_localization_on_start_ && (scan_update_count_ % 10 == 0)) {
      auto est = engine_.get_estimate();
      RCLCPP_INFO(this->get_logger(),
        "Global localization progress: %d updates, current estimate: (%.2f, %.2f, %.2f)",
        scan_update_count_, est.pose.x, est.pose.y, est.pose.theta);
    }

    publish_pose_and_particles(msg->header.stamp); //publishes the estimated pose and particles

    // Also update TF based on current estimate
    auto est = engine_.get_estimate();
    update_map_to_odom_transform(est, msg->header.stamp);
  } else {
    // Keep /mcl_pose and /particlecloud active at ~5 Hz even when stationary
    static int stationary_pub_counter = 0;
    if (++stationary_pub_counter % 6 == 0) {
      publish_pose_and_particles(msg->header.stamp);
    }
  }
}

void MCLNode::update_map_to_odom_transform(const PoseEstimate & est, const rclcpp::Time & stamp)
{
  if (!publish_tf_ || !has_odom_) { //Only publishes the transform if the flag is true and they have odom
    return;
  }

  // T_map_base: robot pose in map frame (from MCL estimate)
  // T_odom_base: robot pose in odom frame (from curr_odom_pose_)
  // T_map_odom = T_map_base * inv(T_odom_base)

  double theta_diff = angle_diff(est.pose.theta, curr_odom_pose_.theta);
  double cos_diff = std::cos(theta_diff);
  double sin_diff = std::sin(theta_diff);

  double map_to_odom_x = est.pose.x - (curr_odom_pose_.x * cos_diff - curr_odom_pose_.y * sin_diff);
  double map_to_odom_y = est.pose.y - (curr_odom_pose_.x * sin_diff + curr_odom_pose_.y * cos_diff);

  map_to_odom_tf_.header.frame_id = global_frame_id_;
  map_to_odom_tf_.child_frame_id = odom_frame_id_;
  // Add a slight future timestamp (50ms) to ensure smooth interpolation by TF listeners
  map_to_odom_tf_.header.stamp = stamp + rclcpp::Duration::from_seconds(0.05);

  map_to_odom_tf_.transform.translation.x = map_to_odom_x;
  map_to_odom_tf_.transform.translation.y = map_to_odom_y;
  map_to_odom_tf_.transform.translation.z = 0.0;

  double qx, qy, qz, qw;
  yaw_to_quaternion(theta_diff, qx, qy, qz, qw);
  map_to_odom_tf_.transform.rotation.x = qx;
  map_to_odom_tf_.transform.rotation.y = qy;
  map_to_odom_tf_.transform.rotation.z = qz;
  map_to_odom_tf_.transform.rotation.w = qw;

  tf_broadcaster_->sendTransform(map_to_odom_tf_);
  has_valid_tf_ = true;
}

void MCLNode::timer_tf_callback()
{
  // Intentionally empty. TF is broadcast synchronously with simulator sensor stamps.
}

void MCLNode::publish_pose_and_particles(const rclcpp::Time & stamp)
{
  auto est = engine_.get_estimate();

  // Publish PoseWithCovarianceStamped
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = global_frame_id_;

  pose_msg.pose.pose.position.x = est.pose.x;
  pose_msg.pose.pose.position.y = est.pose.y;
  pose_msg.pose.pose.position.z = 0.0;

  double qx, qy, qz, qw;
  yaw_to_quaternion(est.pose.theta, qx, qy, qz, qw);
  pose_msg.pose.pose.orientation.x = qx;
  pose_msg.pose.pose.orientation.y = qy;
  pose_msg.pose.pose.orientation.z = qz;
  pose_msg.pose.pose.orientation.w = qw;

  // Fill 6x6 ROS covariance from 3x3 (x, y, yaw)
  // Indices in 6x6 row-major:
  // (0,0)->0, (0,1)->1, (0,5)->5
  // (1,0)->6, (1,1)->7, (1,5)->11
  // (5,0)->30, (5,1)->31, (5,5)->35
  pose_msg.pose.covariance[0] = est.covariance[0];   // cov(x,x)
  pose_msg.pose.covariance[1] = est.covariance[1];   // cov(x,y)
  pose_msg.pose.covariance[5] = est.covariance[2];   // cov(x,yaw)

  pose_msg.pose.covariance[6] = est.covariance[3];   // cov(y,x)
  pose_msg.pose.covariance[7] = est.covariance[4];   // cov(y,y)
  pose_msg.pose.covariance[11] = est.covariance[5];  // cov(y,yaw)

  pose_msg.pose.covariance[30] = est.covariance[6];  // cov(yaw,x)
  pose_msg.pose.covariance[31] = est.covariance[7];  // cov(yaw,y)
  pose_msg.pose.covariance[35] = est.covariance[8];  // cov(yaw,yaw)

  pose_pub_->publish(pose_msg);
  // amcl_pose_pub_->publish(pose_msg);

  // Publish PoseArray particles (downsampled to max 250 poses for RViz to prevent DDS socket buffer overflows)
  const auto & particles = engine_.get_particles();
  if (particlecloud_pub_->get_subscription_count() > 0 && !particles.empty()) {
    const size_t max_viz_particles = 250;
    size_t step = std::max(size_t{1}, particles.size() / max_viz_particles);
    size_t out_count = (particles.size() + step - 1) / step;

    geometry_msgs::msg::PoseArray cloud_msg;
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = global_frame_id_;
    cloud_msg.poses.resize(out_count);

    size_t out_idx = 0;
    for (size_t i = 0; i < particles.size() && out_idx < out_count; i += step, ++out_idx) {
      const auto & p = particles[i];
      auto & pose = cloud_msg.poses[out_idx];
      pose.position.x = p.x;
      pose.position.y = p.y;
      pose.position.z = 0.0;

      double pqx, pqy, pqz, pqw;
      yaw_to_quaternion(p.theta, pqx, pqy, pqz, pqw);
      pose.orientation.x = pqx;
      pose.orientation.y = pqy;
      pose.orientation.z = pqz;
      pose.orientation.w = pqw;
    }

    particlecloud_pub_->publish(cloud_msg);
  }

  // Update and send TF
  update_map_to_odom_transform(est, stamp);
}

}  // namespace montecarlo_localization
