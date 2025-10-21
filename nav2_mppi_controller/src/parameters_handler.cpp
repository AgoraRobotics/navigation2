// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

ParametersHandler::ParametersHandler(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
{
  node_ = parent;
  auto node = node_.lock();
  node_name_ = node->get_name();
  logger_ = node->get_logger();
}

std::string extractParameterName(const std::string& full_param_name) {
  size_t last_dot = full_param_name.find_last_of('.');
  if (last_dot == std::string::npos) {
    return full_param_name;
  }
  return full_param_name.substr(last_dot + 1);
}

void ParametersHandler::start()
{
  auto node = node_.lock();
  on_set_param_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParametersHandler::dynamicParamsCallback, this,
      std::placeholders::_1));

  auto get_param = getParamGetter(node_name_);
  get_param(verbose_, "verbose", false);
}

rcl_interfaces::msg::SetParametersResult
ParametersHandler::dynamicParamsCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock(parameters_change_mutex_);

  for (auto & pre_cb : pre_callbacks_) {
    pre_cb();
  }
  bool reset_everything = false;
  bool found_parameter = false;
  for (auto & param : parameters) {
    const std::string & param_name = param.get_name();
    const std::string actual_param_name = extractParameterName(param_name);
    std::cout << "Full param name: " << param_name << ", Actual name: " << actual_param_name << std::endl;
    reset_everything |= actual_param_name == "time_steps" || actual_param_name == "batch_size";
    if (auto callback = get_param_callbacks_.find(param_name);
      callback != get_param_callbacks_.end())
    {
      found_parameter = true;
      callback->second(param);
    } else {
      RCLCPP_WARN(logger_, "Parameter %s not found", param_name.c_str());
    }
  }

  if (found_parameter) {
    for (auto & post_cb : post_callbacks_) {
      post_cb(reset_everything);
    }
  }
  result.successful = true;
  return result;
}

}  // namespace mppi
