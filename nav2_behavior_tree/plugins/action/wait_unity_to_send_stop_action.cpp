// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>
#include <limits>


#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/plugins/action/wait_unity_to_send_stop_action.hpp"




namespace nav2_behavior_tree
{

using std::placeholders::_1;

WaitUnityToSendStopAction::WaitUnityToSendStopAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  unity_action_received(false)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    getInput("topic_name", topic_name_);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();



    socket_sub_ = node_->create_subscription<std_msgs::msg::String>(
        topic_name_,
        qos,
        std::bind(&WaitUnityToSendStopAction::callbackSocketReceive, this, _1)
        );
}

inline BT::NodeStatus WaitUnityToSendStopAction::tick()
{
    setStatus(BT::NodeStatus::RUNNING);

    //posibil sa trebuiasca sa se faca spin pe node_ ??
    if (unity_action_received) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void
WaitUnityToSendStopAction::callbackSocketReceive(const std_msgs::msg::String::SharedPtr msg)
{
    reader.parse(msg->data, root);
    if (root[0]["response"].asString() == "true") {
        unity_action_received = true;
    }
    
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::WaitUnityToSendStopAction>("WaitUnityToSendStopAction");
}
