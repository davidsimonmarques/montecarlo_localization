#include <rclcpp/rclcpp.hpp>
#include "montecarlo_localization/mhamcl_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<montecarlo_localization::MHAMCLNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
