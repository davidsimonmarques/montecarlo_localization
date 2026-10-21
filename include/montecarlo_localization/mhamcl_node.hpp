#ifndef MONTECARLO_LOCALIZATION_MHAMCL_NODE_HPP_
#define MONTECARLO_LOCALIZATION_MHAMCL_NODE_HPP_

#include "montecarlo_localization/mcl_node.hpp"
#include "montecarlo_localization/mhamcl_engine.hpp"

namespace montecarlo_localization
{

/**
 * @brief ROS 2 Node for MH-AMCL (Adaptive Monte Carlo Localization with Metropolis-Hastings).
 *        Inherits all ROS 2 messaging and TF logic from MCLNode, plugging in MHAMCLEngine.
 */
class MHAMCLNode : public MCLNode
{
public:
  explicit MHAMCLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MHAMCLNode() override = default;

protected:
  void declare_and_get_parameters() override;
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_MHAMCL_NODE_HPP_
