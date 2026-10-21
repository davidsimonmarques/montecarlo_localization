#include "montecarlo_localization/mhmcl_node.hpp"

namespace montecarlo_localization
{

MHMCLNode::MHMCLNode(const rclcpp::NodeOptions & options)
: MCLNode("mhmcl_node", options, false)
{
  declare_and_get_parameters();
  setup_node();
}

void MHMCLNode::declare_and_get_parameters()
{
  // Frame IDs
  this->declare_parameter<std::string>("global_frame_id", "map");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_link");

  // Topics
  this->declare_parameter<std::string>("scan_topic", "/scan");
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<std::string>("odom_topic", "/odom");
  this->declare_parameter<std::string>("pose_topic", "/mhmcl_pose");
  this->declare_parameter<std::string>("particlecloud_topic", "/mhmcl_particlecloud");

  // TF Publishing
  this->declare_parameter<bool>("publish_tf", true);

  // Filter Particle Parameters
  this->declare_parameter<int>("num_particles", 8000);
  this->declare_parameter<double>("update_min_d", 0.1);
  this->declare_parameter<double>("update_min_a", 0.1);
  this->declare_parameter<bool>("resample_on_update", true);

  // Odometry Motion Model Noise Parameters
  this->declare_parameter<double>("alpha1", 0.02);
  this->declare_parameter<double>("alpha2", 0.02);
  this->declare_parameter<double>("alpha3", 0.02);
  this->declare_parameter<double>("alpha4", 0.02);

  // Sensor Model Parameters
  this->declare_parameter<double>("z_hit", 0.85);
  this->declare_parameter<double>("z_rand", 0.15);
  this->declare_parameter<double>("sigma_hit", 0.25);
  this->declare_parameter<double>("min_range", 0.1);
  this->declare_parameter<double>("max_range", 5.0);
  this->declare_parameter<int>("beam_skip", 2);
  this->declare_parameter<int>("max_beams", 30);

  // Recovery Parameters (Algorithm 3, lines 19-20: Simple Resampling for MH-MCL)
  this->declare_parameter<double>("alpha_slow", 0.001);
  this->declare_parameter<double>("alpha_fast", 0.1);
  this->declare_parameter<bool>("enable_recovery", false);
  this->declare_parameter<double>("recovery_max_prob", 0.2);

  // Initial Pose
  this->declare_parameter<bool>("auto_initialize", true);
  this->declare_parameter<bool>("global_localization_on_start", false);
  this->declare_parameter<double>("initial_pose_x", 11.16);
  this->declare_parameter<double>("initial_pose_y", -8.77);
  this->declare_parameter<double>("initial_pose_yaw", -3.1232);
  this->declare_parameter<double>("initial_cov_x", 0.25);
  this->declare_parameter<double>("initial_cov_y", 0.25);
  this->declare_parameter<double>("initial_cov_yaw", 0.068);

  // Retrieve parameters
  global_frame_id_ = this->get_parameter("global_frame_id").as_string();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  scan_topic_ = this->get_parameter("scan_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  pose_topic_ = this->get_parameter("pose_topic").as_string();
  particlecloud_topic_ = this->get_parameter("particlecloud_topic").as_string();
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

  // Instantiate MHMCLEngine
  engine_ = std::make_unique<MHMCLEngine>(params);
}

}  // namespace montecarlo_localization
