#include <rclcpp/rclcpp.hpp>
#include "montecarlo_localization/mcl_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<montecarlo_localization::MCLNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
