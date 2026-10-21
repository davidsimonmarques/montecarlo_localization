#ifndef MONTECARLO_LOCALIZATION_AMCL_NODE_HPP_
#define MONTECARLO_LOCALIZATION_AMCL_NODE_HPP_

#include <memory>
#include <string>
#include "montecarlo_localization/mcl_node.hpp"
#include "montecarlo_localization/amcl_engine.hpp"

namespace montecarlo_localization
{

/**
 * @brief ROS 2 Node for Adaptive Monte Carlo Localization (AMCL).
 *        Inherits all ROS 2 messaging, TF broadcasting, and pipeline logic from MCLNode,
 *        while plugging in the AMCLEngine implementing Dieter Fox's KLD-Sampling algorithm.
 */
class AMCLNode : public MCLNode
{
public:
  explicit AMCLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AMCLNode() override = default;

protected:
  void declare_and_get_parameters() override;
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_AMCL_NODE_HPP_
