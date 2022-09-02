#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/complete_coverage_service.hpp"

namespace nav2_behavior_tree
{

CompleteCoverageService::CompleteCoverageService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::CompleteCoverageArray>(service_node_name, conf)
{
}

void CompleteCoverageService::on_tick()
{
  increment_recovery_count();
}


BT::NodeStatus CompleteCoverageService::on_completion()
{
    setOutput("output_path", future_result_.get()->path);
    return BT::NodeStatus::SUCCESS;
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CompleteCoverageService>("CompleteCoverage");
}
