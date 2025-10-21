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

#include "nav2_mppi_controller/critics/agora_path_follow_critic.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>

namespace mppi::critics
{

void AgoraPathFollowCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 1.4);
  getParam(path_carrot_index_, "path_carrot_index", 6);
  getParam(trajectory_offset_, "trajectory_offset", -1);
  getParam(carrot_dist_, "carrot_dist", 0.7);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);
}

void AgoraPathFollowCritic::score(CriticData & data)
{
  if (!enabled_ || data.path.x.shape(0) < 2 ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  const int path_size = data.path.x.shape(0) - 1;

  // carrot that's slightly further in euclidean distance from the robot
  auto offseted_idx = std::min(path_carrot_index_, path_size);
  const double distance_x = data.state.pose.pose.position.x - data.path.x[0];
  const double distance_y = data.state.pose.pose.position.y - data.path.y[0];
  if (sqrt(distance_x * distance_x + distance_y * distance_y) >= carrot_dist_) {
    offseted_idx = 0;
  }
  // Drive to the first valid path point, in case of dynamic obstacles on path
  // we want to drive past it, not through it
  bool valid = false;
  while (!valid && offseted_idx < path_size - 1) {
    valid = (*data.path_pts_valid)[offseted_idx];
    if (!valid) {
      offseted_idx++;
    }
  }
  const auto path_x = data.path.x(offseted_idx);
  const auto path_y = data.path.y(offseted_idx);

  int chosen_offset = trajectory_offset_;
  if (chosen_offset >= (int)data.trajectories.x.shape()[1]) {
    chosen_offset = -1;
  }
  const auto offset_x = xt::view(data.trajectories.x, xt::all(), chosen_offset);
  const auto offset_y = xt::view(data.trajectories.y, xt::all(), chosen_offset);

  auto dists = xt::sqrt(
    xt::pow(offset_x - path_x, 2) +
    xt::pow(offset_y - path_y, 2));

  data.costs += xt::pow(weight_ * std::move(dists), power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::AgoraPathFollowCritic,
  mppi::critics::CriticFunction)