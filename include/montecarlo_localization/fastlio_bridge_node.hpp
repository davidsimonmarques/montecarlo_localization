#ifndef MONTECARLO_LOCALIZATION_FASTLIO_BRIDGE_NODE_HPP_
#define MONTECARLO_LOCALIZATION_FASTLIO_BRIDGE_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace montecarlo_localization
{

class FastlioBridgeNode : public rclcpp::Node
{
public:
  explicit FastlioBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FastlioBridgeNode() override = default;

private:
  void declare_and_get_parameters();
  void precompute_ground_ring();
  void publish_static_odom_to_fastlio_world();

  void try_resolve_laser_tf(const std::string & frame_id);
  void try_resolve_imu_tf(const std::string & frame_id);

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void fastlio_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void broadcast_odom_to_base_tf(const tf2::Transform & tf_odom_base, const rclcpp::Time & stamp);

  static std::string strip_leading_slash(const std::string & frame_id);

  // Topics
  std::string scan_topic_{"/scan"};
  std::string imu_topic_{"/imu/data"};
  std::string cloud_out_topic_{"/fastlio2/scan_cloud"};
  std::string imu_out_topic_{"/fastlio2/imu"};
  std::string fastlio_odom_in_topic_{"/Odometry"};
  std::string odom_out_topic_{"/fastlio2/odom"};

  // Frame IDs
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_footprint"};
  std::string fastlio_world_frame_id_{"camera_init"};

  // Pseudo-3D cloud generation settings for 2D LaserScan -> 3D FAST-LIO2
  bool publish_odom_tf_{true};
  int vertical_layers_{5};
  double vertical_z_min_{-0.30};
  double vertical_z_max_{0.30};
  bool enable_ground_plane_{true};
  double ground_z_{-0.60};
  double min_range_{0.15};
  double max_range_{9.95};
  double max_range_margin_{0.05};
  double stamp_offset_sec_{0.001};

  // IMU gravity check for simulated IMUs that omit +g reaction acceleration
  bool ensure_imu_gravity_{true};
  double gravity_mps2_{9.81};
  bool gravity_checked_{false};
  bool inject_gravity_{false};
  int imu_init_samples_{0};

  // Extrinsics (base_link -> sensor)
  tf2::Transform t_base_laser_;
  tf2::Transform t_base_imu_;
  bool laser_tf_resolved_{false};
  bool imu_tf_resolved_{false};
  int imu_tf_lookup_attempts_{0};
  int laser_tf_lookup_attempts_{0};

  // Precomputed horizontal ground points (x, y) in base_link frame
  std::vector<std::pair<float, float>> ground_xy_points_;
  std::vector<float> vertical_z_offsets_;

  // Timestamp monotonicity tracking
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_tf_stamp_{0, 0, RCL_ROS_TIME};
  bool has_fastlio_odom_{false};

  // TF2 interfaces
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // ROS 2 Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_FASTLIO_BRIDGE_NODE_HPP_
