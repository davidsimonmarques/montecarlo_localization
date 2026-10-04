#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

class GoalRelayNode : public rclcpp::Node {
public:
  GoalRelayNode() : Node("goal_relay_node") {
    // Publisher on /goal_position (expected by navigation algorithms like RRT*, Wavefront, etc.)
    pub_point_ = create_publisher<geometry_msgs::msg::Point>("/goal_position", 10);

    // Subscriber on /goal_pose (published by RViz "2D Goal Pose" tool)
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        geometry_msgs::msg::Point pt;
        pt.x = msg->pose.position.x;
        pt.y = msg->pose.position.y;
        pt.z = msg->pose.position.z;
        RCLCPP_INFO(get_logger(),
          "Relaying RViz /goal_pose (X: %.2f, Y: %.2f) -> /goal_position",
          pt.x, pt.y);
        pub_point_->publish(pt);
      });

    RCLCPP_INFO(get_logger(),
      "Goal relay active: listening to /goal_pose and forwarding to /goal_position");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_point_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalRelayNode>());
  rclcpp::shutdown();
  return 0;
}
