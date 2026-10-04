#ifndef MONTECARLO_LOCALIZATION_MCL_NODE_HPP_
#define MONTECARLO_LOCALIZATION_MCL_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "montecarlo_localization/mcl_engine.hpp"

namespace montecarlo_localization
{

class MCLNode : public rclcpp::Node
{
public:
  explicit MCLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MCLNode() override = default;

private:
  void declare_and_get_parameters();
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void timer_tf_callback();

  void update_laser_offset_from_tf(const std::string & laser_frame);
  void publish_pose_and_particles(const rclcpp::Time & stamp);
  void update_map_to_odom_transform(const PoseEstimate & est, const rclcpp::Time & stamp);

  // Engine instance
  MCLEngine engine_;

  // Frame IDs
  std::string global_frame_id_{"map"};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};

  // Topics
  std::string scan_topic_{"/scan"};
  std::string map_topic_{"/map"};
  std::string odom_topic_{"/odom"};

  // Odometry tracking
  bool has_odom_{false};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  Pose2D prev_odom_pose_;
  Pose2D curr_odom_pose_;
  Pose2D last_update_odom_pose_;

  // TF
  bool publish_tf_{true};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped map_to_odom_tf_;
  bool has_valid_tf_{false};
  bool laser_offset_resolved_{false};

  // Subscriptions & Publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;

  rclcpp::TimerBase::SharedPtr tf_timer_;

  // Initialization settings
  bool auto_initialize_{true};
  bool global_localization_on_start_{false};
  bool force_initial_update_{true};  // Force sensor update even without movement after init
  int scan_update_count_{0};         // Track number of sensor updates performed
  double init_x_{0.0};
  double init_y_{0.0};
  double init_yaw_{0.0};
  double init_cov_x_{0.5};
  double init_cov_y_{0.5};
  double init_cov_yaw_{0.25};
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_MCL_NODE_HPP_
