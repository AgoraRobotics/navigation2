#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPLETE_COVERAGE_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPLETE_COVERAGE_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/srv/complete_coverage.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps nav2_msgs::srv::CompleteCoverage
 */
class CompleteCoverageService : public BtServiceNode<nav2_msgs::srv::CompleteCoverage>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::CompleteCoverageService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  CompleteCoverageService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;


  BT::NodeStatus on_completion() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>("output_path", "Coverage planning path"),
      });
  }

};


}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPLETE_COVERAGE_SERVICE_HPP_
