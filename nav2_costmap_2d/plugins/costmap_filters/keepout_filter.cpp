/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/

#include <string>
#include <memory>
#include <algorithm>
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_costmap_2d
{

KeepoutFilter::KeepoutFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr), mask_costmap_(nullptr),
  mask_frame_(""), global_frame_("")
{
}

void KeepoutFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declareParameter("footprint_padding", rclcpp::ParameterValue(-0.03));
  node->get_parameter(name_ + "." + "footprint_padding", footprint_padding);

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  RCLCPP_INFO(
    logger_,
    "KeepoutFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::filterInfoCallback, this, std::placeholders::_1));

  global_frame_ = layered_costmap_->getGlobalFrameID();
}

void KeepoutFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_,
      "KeepoutFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    mask_sub_.reset();
  }

  // Checking that base and multiplier are set to their default values
  if (msg->base != BASE_DEFAULT || msg->multiplier != MULTIPLIER_DEFAULT) {
    RCLCPP_ERROR(
      logger_,
      "KeepoutFilter: For proper use of keepout filter base and multiplier"
      " in CostmapFilterInfo message should be set to their default values (%f and %f)",
      BASE_DEFAULT, MULTIPLIER_DEFAULT);
  }

  mask_topic_ = msg->filter_mask_topic;

  // Setting new filter mask subscriber
  RCLCPP_INFO(
    logger_,
    "KeepoutFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::maskCallback, this, std::placeholders::_1));
}

void KeepoutFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_costmap_) {
    RCLCPP_INFO(
      logger_,
      "KeepoutFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    mask_costmap_.reset();
  }

  // Making a new mask_costmap_
  mask_costmap_ = std::make_unique<Costmap2D>(*msg);
  mask_frame_ = msg->header.frame_id;
}

std::vector<geometry_msgs::msg::Point> KeepoutFilter::inflatePolygon(const std::vector<geometry_msgs::msg::Point> & footprint, const double inflation_distance) {
  const size_t n = footprint.size();
  std::vector<geometry_msgs::msg::Point> inflated_polygon(n);

  for (size_t i = 0; i < n; ++i) {
      // Get previous, current, and next vertex
      geometry_msgs::msg::Point p_prev = footprint[(i + n - 1) % n];
      geometry_msgs::msg::Point p_curr = footprint[i];
      geometry_msgs::msg::Point p_next = footprint[(i + 1) % n];

      // Compute two edge vectors
      double edge1_x = p_curr.x - p_prev.x;
      double edge1_y = p_curr.y - p_prev.y;
      double edge2_x = p_next.x - p_curr.x;
      double edge2_y = p_next.y - p_curr.y;

      // Normalize edge vectors
      const double len1 = std::sqrt(edge1_x * edge1_x + edge1_y * edge1_y);
      const double len2 = std::sqrt(edge2_x * edge2_x + edge2_y * edge2_y);

      if (len1 > 0) { edge1_x /= len1; edge1_y /= len1; }
      if (len2 > 0) { edge2_x /= len2; edge2_y /= len2; }

      // Compute the average normal (rotated 90 degrees outward)
      double normal_x = (edge1_y + edge2_y);
      double normal_y = -(edge1_x + edge2_x);

      // Normalize the normal vector
      const double normal_length = std::sqrt(normal_x * normal_x + normal_y * normal_y);
      if (normal_length > 0) {
          normal_x = (normal_x / normal_length) * inflation_distance;
          normal_y = (normal_y / normal_length) * inflation_distance;
      }

      // Apply inflation to the vertex
      inflated_polygon[i].x = p_curr.x + normal_x;
      inflated_polygon[i].y = p_curr.y + normal_y;
  }


  return inflated_polygon;
}


bool KeepoutFilter::isPointInsideRobot(const double msk_wx, const double msk_wy, const std::vector<geometry_msgs::msg::Point> & transformed_footprint_)
{
  const size_t n = transformed_footprint_.size();

  // Determine the reference sign using the first edge
  geometry_msgs::msg::Point p1 = transformed_footprint_[0];
  geometry_msgs::msg::Point p2 = transformed_footprint_[1];
  const double ref_cross = (p2.x - p1.x) * (msk_wy - p1.y) - (p2.y - p1.y) * (msk_wx - p1.x);

  for (size_t i = 1; i < n; ++i) {
      p1 = transformed_footprint_[i];
      p2 = transformed_footprint_[(i + 1) % n];

      const double cross = (p2.x - p1.x) * (msk_wy - p1.y) - (p2.y - p1.y) * (msk_wx - p1.x);

      if ((ref_cross > 0 && cross < 0) || (ref_cross < 0 && cross > 0)) {
          return false; // The point is outside if it is not consistently on the same side.
      }
  }

  return true; // The point is inside if all cross products have the same sign.
}


void KeepoutFilter::process(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const geometry_msgs::msg::Pose2D & pose)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!mask_costmap_) {
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "KeepoutFilter: Filter mask was not received");
    return;
  }

  auto footprint = getFootprint();

  if (footprint.size() < 3) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "KeepoutFilter: Robot footprint must have at least 3 points");
    return;
  }

  // inflation with half a pixel (1 pixel = 5cm) to overcome rounding errors
  auto inflated_footprint = inflatePolygon(footprint, footprint_padding);
  transformFootprint(pose.x, pose.y, pose.theta, inflated_footprint, transformed_footprint_);

  tf2::Transform tf2_transform;
  tf2_transform.setIdentity();  // initialize by identical transform
  int mg_min_x, mg_min_y;  // masger_grid indexes of bottom-left window corner
  int mg_max_x, mg_max_y;  // masger_grid indexes of top-right window corner

  if (mask_frame_ != global_frame_) {
    // Filter mask and current layer are in different frames:
    // prepare frame transformation if mask_frame_ != global_frame_
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        mask_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "KeepoutFilter: Failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        global_frame_.c_str(), mask_frame_.c_str(), ex.what());
      return;
    }
    tf2::fromMsg(transform.transform, tf2_transform);

    mg_min_x = min_i;
    mg_min_y = min_j;
    mg_max_x = max_i;
    mg_max_y = max_j;
  } else {
    // Filter mask and current layer are in the same frame:
    // apply the following optimization - iterate only in overlapped
    // (min_i, min_j)..(max_i, max_j) & mask_costmap_ area.
    //
    //           mask_costmap_
    //       *----------------------------*
    //       |                            |
    //       |                            |
    //       |      (2)                   |
    // *-----+-------*                    |
    // |     |///////|<- overlapped area  |
    // |     |///////|   to iterate in    |
    // |     *-------+--------------------*
    // |    (1)      |
    // |             |
    // *-------------*
    //  master_grid (min_i, min_j)..(max_i, max_j) window
    //
    // ToDo: after costmap rotation will be added, this should be re-worked.

    double wx, wy;  // world coordinates

    // Calculating bounds corresponding to bottom-left overlapping (1) corner
    // mask_costmap_ -> master_grid intexes conversion
    const double half_cell_size = 0.5 * mask_costmap_->getResolution();
    wx = mask_costmap_->getOriginX() + half_cell_size;
    wy = mask_costmap_->getOriginY() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);
    // Calculation of (1) corner bounds
    if (mg_min_x >= max_i || mg_min_y >= max_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_min_x = std::max(min_i, mg_min_x);
    mg_min_y = std::max(min_j, mg_min_y);

    // Calculating bounds corresponding to top-right window (2) corner
    // mask_costmap_ -> master_grid intexes conversion
    wx = mask_costmap_->getOriginX() +
      mask_costmap_->getSizeInCellsX() * mask_costmap_->getResolution() + half_cell_size;
    wy = mask_costmap_->getOriginY() +
      mask_costmap_->getSizeInCellsY() * mask_costmap_->getResolution() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);
    // Calculation of (2) corner bounds
    if (mg_max_x <= min_i || mg_max_y <= min_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_max_x = std::min(max_i, mg_max_x);
    mg_max_y = std::min(max_j, mg_max_y);
  }

  // unsigned<-signed conversions.
  unsigned const int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
  unsigned const int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
  unsigned const int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
  unsigned const int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

  unsigned int i, j;  // master_grid iterators
  unsigned int index;  // corresponding index of master_grid
  double gl_wx, gl_wy;  // world coordinates in a global_frame_
  double msk_wx, msk_wy;  // world coordinates in a mask_frame_
  unsigned int mx, my;  // mask_costmap_ coordinates
  unsigned char data, old_data;  // master_grid element data

  // Main master_grid updating loop
  // Iterate in costmap window by master_grid indexes
  unsigned char * master_array = master_grid.getCharMap();
  for (i = mg_min_x_u; i < mg_max_x_u; i++) {
    for (j = mg_min_y_u; j < mg_max_y_u; j++) {
      index = master_grid.getIndex(i, j);
      old_data = master_array[index];
      // Calculating corresponding to (i, j) point at mask_costmap_:
      // Get world coordinates in global_frame_
      master_grid.mapToWorld(i, j, gl_wx, gl_wy);
      if (mask_frame_ != global_frame_) {
        // Transform (i, j) point from global_frame_ to mask_frame_
        tf2::Vector3 point(gl_wx, gl_wy, 0);
        point = tf2_transform * point;
        msk_wx = point.x();
        msk_wy = point.y();
      } else {
        // In this case master_grid and filter-mask are in the same frame
        msk_wx = gl_wx;
        msk_wy = gl_wy;
      }
      // Get mask coordinates corresponding to (i, j) point at mask_costmap_
      if (mask_costmap_->worldToMap(msk_wx, msk_wy, mx, my)) {
        data = mask_costmap_->getCost(mx, my);
        // Update if mask_ data is valid and greater than existing master_grid's one
        if (data == NO_INFORMATION) {
          continue;
        }
        if (data > old_data || old_data == NO_INFORMATION) {
          master_array[index] = data;
        }
        // check if point is inside robot footprint
        if (data == LETHAL_OBSTACLE && isPointInsideRobot(msk_wx, msk_wy, transformed_footprint_)) {
            master_array[index] = old_data;
        }
      }
    }
  }
}

void KeepoutFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
}

bool KeepoutFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (mask_costmap_) {
    return true;
  }
  return false;
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::KeepoutFilter, nav2_costmap_2d::Layer)
