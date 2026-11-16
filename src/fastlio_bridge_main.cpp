#include <rclcpp/rclcpp.hpp>

#include "montecarlo_localization/fastlio_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<montecarlo_localization::FastlioBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
