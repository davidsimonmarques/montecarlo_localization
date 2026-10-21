#ifndef MONTECARLO_LOCALIZATION_MHMCL_NODE_HPP_
#define MONTECARLO_LOCALIZATION_MHMCL_NODE_HPP_

#include "montecarlo_localization/mcl_node.hpp"
#include "montecarlo_localization/mhmcl_engine.hpp"

namespace montecarlo_localization
{

/**
 * @brief ROS 2 Node for MH-MCL (Monte Carlo Localization with Metropolis-Hastings).
 *        Inherits all ROS 2 communication, TF publishing, and timing from MCLNode,
 *        plugging in MHMCLEngine.
 */
class MHMCLNode : public MCLNode
{
public:
  explicit MHMCLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MHMCLNode() override = default;

protected:
  void declare_and_get_parameters() override;
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_MHMCL_NODE_HPP_
