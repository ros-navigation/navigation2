/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2019, Samsung Research America, Inc.
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
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"

#include <string>
#include <memory>

#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_costmap_2d
{

Costmap2DPublisher::Costmap2DPublisher(
  nav2_util::LifecycleNode::SharedPtr ros_node, Costmap2D * costmap,
  std::string global_frame,
  std::string topic_name,
  bool always_send_full_costmap)
: node_(ros_node), costmap_(costmap), global_frame_(global_frame), topic_name_(topic_name),
  active_(false), always_send_full_costmap_(always_send_full_costmap)
{
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  // TODO(bpwilcox): port onNewSubscription functionality for publisher
  costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name,
      custom_qos);
  costmap_raw_pub_ = node_->create_publisher<nav2_msgs::msg::Costmap>(topic_name + "_raw",
      custom_qos);
  costmap_update_pub_ = node_->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
    topic_name + "_updates", custom_qos);

  // Create a service that will use the callback function to handle requests.
  costmap_service_ = node_->create_service<nav2_msgs::srv::GetCostmap>(
    "get_costmap", std::bind(&Costmap2DPublisher::costmap_service_callback,
    this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3));

  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

Costmap2DPublisher::~Costmap2DPublisher() {}

// TODO(bpwilcox): find equivalent/workaround to ros::SingleSubscriberPublishr
/*
void Costmap2DPublisher::onNewSubscription(const ros::SingleSubscriberPublisher& pub)
{
  prepareGrid();
  pub.publish(grid_);
} */

// prepare grid_ message for publication.
void Costmap2DPublisher::prepareGrid()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  grid_ = toOccupancyGridMsg(costmap_, global_frame_);
}

void Costmap2DPublisher::prepareCostmap()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  costmap_raw_ = toCostmapMsg(costmap_, global_frame_, node_->now());
}

void Costmap2DPublisher::publishCostmap()
{
  if (node_->count_subscribers(costmap_raw_pub_->get_topic_name()) > 0) {
    prepareCostmap();
    costmap_raw_pub_->publish(costmap_raw_);
  }
  float resolution = costmap_->getResolution();

  if (always_send_full_costmap_ || grid_.info.resolution != resolution ||
    grid_.info.width != costmap_->getSizeInCellsX() ||
    grid_.info.height != costmap_->getSizeInCellsY() ||
    saved_origin_x_ != costmap_->getOriginX() ||
    saved_origin_y_ != costmap_->getOriginY())
  {
    if (node_->count_subscribers(costmap_pub_->get_topic_name()) > 0) {
      prepareGrid();
      costmap_pub_->publish(grid_);
    }
  } else if (x0_ < xn_) {
    if (node_->count_subscribers(costmap_update_pub_->get_topic_name()) > 0) {
      std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
      // Publish Just an Update
      map_msgs::msg::OccupancyGridUpdate update =
        toOccupancyGridUpdateMsg(costmap_, global_frame_, x0_, xn_, y0_, yn_);
      costmap_update_pub_->publish(update);
    }
  }

  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

void
Costmap2DPublisher::costmap_service_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request>/*request*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response)
{
  RCLCPP_DEBUG(node_->get_logger(), "Received costmap service request");

  // TODO(bpwilcox): Grab correct orientation information
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.0);

  auto size_x = costmap_->getSizeInCellsX();
  auto size_y = costmap_->getSizeInCellsY();
  auto data_length = size_x * size_y;
  unsigned char * data = costmap_->getCharMap();
  auto current_time = node_->now();

  response->map.header.stamp = current_time;
  response->map.header.frame_id = global_frame_;
  response->map.metadata.size_x = size_x;
  response->map.metadata.size_y = size_y;
  response->map.metadata.resolution = costmap_->getResolution();
  response->map.metadata.layer = "master";
  response->map.metadata.map_load_time = current_time;
  response->map.metadata.update_time = current_time;
  response->map.metadata.origin.position.x = costmap_->getOriginX();
  response->map.metadata.origin.position.y = costmap_->getOriginY();
  response->map.metadata.origin.position.z = 0.0;
  response->map.metadata.origin.orientation = tf2::toMsg(quaternion);
  response->map.data.resize(data_length);
  response->map.data.assign(data, data + data_length);
}

}  // end namespace nav2_costmap_2d
