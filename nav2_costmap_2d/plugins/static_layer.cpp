/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <nav2_costmap_2d/static_layer.h>
#include <nav2_costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::StaticLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_costmap_2d
{

StaticLayer::StaticLayer() {enabled_ = true;}

StaticLayer::~StaticLayer()
{
  if (dynamic_param_client_) {
    delete dynamic_param_client_;
  }
}

void StaticLayer::onInitialize()
{
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  node_->set_parameter_if_not_set("enabled_static_layer",true);

  std::string map_topic;
  node_->get_parameter_or<std::string>("map_topic", map_topic, std::string("occ_grid"));
  node_->get_parameter_or<bool>("first_map_only", first_map_only_, false);
  node_->get_parameter_or<bool>("subscribe_to_updates", subscribe_to_updates_, false);

  node_->get_parameter_or<bool>("track_unknown_space", track_unknown_space_, true);
  node_->get_parameter_or<bool>("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  node_->get_parameter_or<int>("lethal_cost_threshold", temp_lethal_threshold, 100);
  node_->get_parameter_or<int>("unknown_cost_value", temp_unknown_cost_value, -1);
  node_->get_parameter_or<bool>("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  // Only resubscribe if topic has changed
  //TODO(bpwilcox): resolve ROS2 equivalent to check whether topic has changed. Problem calling get_topic before topic created
  //if (map_sub_.getTopic() != ros::names::resolve(map_topic)) {

  // we'll subscribe to the latched topic that the map server uses
  RCLCPP_INFO(node_->get_logger(), "Requesting the map...");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 1;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic,
      std::bind(&StaticLayer::incomingMap, this, std::placeholders::_1), custom_qos_profile);
  map_received_ = false;
  has_updated_data_ = false;

  rclcpp::Rate r(10);
  rclcpp::executors::SingleThreadedExecutor exec;
  while (!map_received_ && rclcpp::ok()) {
    exec.spin_node_once(node_->get_node_base_interface(), std::chrono::milliseconds(100));
    r.sleep();
  }

  RCLCPP_INFO(node_->get_logger(),
    "Received a %d X %d map at %f m/pix", getSizeInCellsX(),
      getSizeInCellsY(), getResolution());

  if (subscribe_to_updates_) {
    RCLCPP_INFO(node_->get_logger(), "Subscribing to updates");
    map_update_sub_ = node_->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        map_topic + "_updates",
        std::bind(&StaticLayer::incomingUpdate, this, std::placeholders::_1), custom_qos_profile);

  } else {
    has_updated_data_ = true;
  }

  dynamic_param_client_ = new nav2_dynamic_params::DynamicParamsClient(node_);
  dynamic_param_client_->set_callback(std::bind(&StaticLayer::reconfigureCB, this, std::placeholders::_1));
  // TODO(bpwilcox): Add new parameters to parameter validation class from plugins
  // TODO(bpwilcox): Initialize callback for dynamic parameters
}

void StaticLayer::reconfigureCB(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  RCLCPP_DEBUG(node_->get_logger(), "StaticLayer:: Event Callback");

  bool enabled = true;
  dynamic_param_client_->get_event_param_or(event,"enabled_static_layer", enabled, true); 

  if (enabled != enabled_)
  {
    enabled_ = enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling()) {
    Costmap2D * master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
        master->getOriginX(), master->getOriginY());
  }
}

unsigned char StaticLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_) {
    return NO_INFORMATION;
  } else if (!track_unknown_space_ && value == unknown_cost_value_) {
    return FREE_SPACE;
  } else if (value >= lethal_threshold_) {
    return LETHAL_OBSTACLE;
  } else if (trinary_costmap_) {
    return FREE_SPACE;
  }

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  RCLCPP_DEBUG(node_->get_logger(),
    "Received a %d X %d map at %f m/pix", size_x, size_y,
      new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D * master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
        master->getSizeInCellsY() != size_y ||
        master->getResolution() != new_map->info.resolution ||
        master->getOriginX() != new_map->info.origin.position.x ||
        master->getOriginY() != new_map->info.origin.position.y ||
        !layered_costmap_->isSizeLocked()))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    RCLCPP_INFO(node_->get_logger(),
      "Resizing costmap to %d X %d at %f m/pix", size_x, size_y,
        new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution,
        new_map->info.origin.position.x,
        new_map->info.origin.position.y,
        true);
  } else if (size_x_ != size_x || size_y_ != size_y ||
      resolution_ != new_map->info.resolution ||
      origin_x_ != new_map->info.origin.position.x ||
      origin_y_ != new_map->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    RCLCPP_INFO(node_->get_logger(),
      "Resizing static layer to %d X %d at %f m/pix", size_x, size_y,
        new_map->info.resolution);
    resizeMap(size_x, size_y, new_map->info.resolution,
        new_map->info.origin.position.x, new_map->info.origin.position.y);
  }

  unsigned int index = 0;

  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }

  map_frame_ = new_map->header.frame_id;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_) {
    RCLCPP_INFO(node_->get_logger(),
      "Shutting down the map subscriber. first_map_only flag is on");
    // TODO(bpwilcox): Resolve shutdown of ros2 subscription
    //map_sub_.shutdown();
  }
}

void StaticLayer::incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height; y++) {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width; x++) {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void StaticLayer::activate()
{
  onInitialize();
}

void StaticLayer::deactivate()
{
  // TODO(bpwilcox): Resolve shutdown of ros2 subscription
  //map_sub_.shutdown();
  if (subscribe_to_updates_) {
    //map_update_sub_.shutdown();
  }
}

void StaticLayer::reset()
{
  if (first_map_only_) {
    has_updated_data_ = true;
  } else {
    onInitialize();
  }
}

void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y)
{

  if (!layered_costmap_->isRolling() ) {
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}

void StaticLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j)
{
  if (!map_received_) {
    return;
  }

  if (!enabled_) {
    return;
  }

  if (!layered_costmap_->isRolling()) {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!use_maximum_) {
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    } else {
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(map_frame_, global_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException ex) {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::fromMsg(transform.transform, tf2_transform);

    for (unsigned int i = min_i; i < max_i; ++i) {
      for (unsigned int j = min_j; j < max_j; ++j) {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform * p;
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my)) {
          if (!use_maximum_) {
            master_grid.setCost(i, j, getCost(mx, my));
          } else {
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
          }
        }
      }
    }
  }
}

}  // namespace nav2_costmap_2d
