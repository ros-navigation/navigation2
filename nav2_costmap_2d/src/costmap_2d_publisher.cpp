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
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_costmap_2d
{

char * Costmap2DPublisher::cost_translation_table_ = NULL;

Costmap2DPublisher::Costmap2DPublisher(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  Costmap2D * costmap,
  std::string global_frame,
  std::string topic_name,
  bool always_send_full_costmap,
  double map_vis_z)
: costmap_(costmap),
  global_frame_(global_frame),
  topic_name_(topic_name),
  active_(false),
  always_send_full_costmap_(always_send_full_costmap),
  map_vis_z_(map_vis_z)
{
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  // TODO(bpwilcox): port onNewSubscription functionality for publisher
  costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name,
    custom_qos);
  costmap_raw_pub_ = node->create_publisher<nav2_msgs::msg::Costmap>(
    topic_name + "_raw",
    custom_qos);
  costmap_update_pub_ = node->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
    topic_name + "_updates", custom_qos);
  costmap_raw_update_pub_ = node->create_publisher<nav2_msgs::msg::CostmapUpdate>(
    topic_name + "_raw_updates", custom_qos);

  // Create a service that will use the callback function to handle requests.
  costmap_service_ = node->create_service<nav2_msgs::srv::GetCostmap>(
    "get_" + topic_name, std::bind(
      &Costmap2DPublisher::costmap_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  if (cost_translation_table_ == NULL) {
    cost_translation_table_ = new char[256];

    // special values:
    cost_translation_table_[0] = 0;  // NO obstacle
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle
    cost_translation_table_[254] = 100;  // LETHAL obstacle
    cost_translation_table_[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++) {
      cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
    }
  }

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


void Costmap2DPublisher::updateGridParams()
{
  saved_origin_x_ = costmap_->getOriginX();
  saved_origin_y_ = costmap_->getOriginY();
  grid_resolution_ = costmap_->getResolution();
  grid_width_ = costmap_->getSizeInCellsX();
  grid_height_ = costmap_->getSizeInCellsY();
}

// prepare grid_ message for publication.
void Costmap2DPublisher::prepareGrid()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  grid_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  grid_->header.frame_id = global_frame_;
  grid_->header.stamp = clock_->now();

  grid_->info.resolution = grid_resolution_;

  grid_->info.width = grid_width_;
  grid_->info.height = grid_height_;

  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  grid_->info.origin.position.x = wx - grid_resolution_ / 2;
  grid_->info.origin.position.y = wy - grid_resolution_ / 2;
  grid_->info.origin.position.z = map_vis_z_;
  grid_->info.origin.orientation.w = 1.0;

  grid_->data.resize(grid_->info.width * grid_->info.height);

  unsigned char * data = costmap_->getCharMap();
  for (unsigned int i = 0; i < grid_->data.size(); i++) {
    grid_->data[i] = cost_translation_table_[data[i]];
  }
}

void Costmap2DPublisher::prepareCostmap()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  double resolution = costmap_->getResolution();

  costmap_raw_ = std::make_unique<nav2_msgs::msg::Costmap>();

  costmap_raw_->header.frame_id = global_frame_;
  costmap_raw_->header.stamp = clock_->now();

  costmap_raw_->metadata.layer = "master";
  costmap_raw_->metadata.resolution = resolution;

  costmap_raw_->metadata.size_x = costmap_->getSizeInCellsX();
  costmap_raw_->metadata.size_y = costmap_->getSizeInCellsY();

  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  costmap_raw_->metadata.origin.position.x = wx - resolution / 2;
  costmap_raw_->metadata.origin.position.y = wy - resolution / 2;
  costmap_raw_->metadata.origin.position.z = 0.0;
  costmap_raw_->metadata.origin.orientation.w = 1.0;

  costmap_raw_->data.resize(costmap_raw_->metadata.size_x * costmap_raw_->metadata.size_y);

  unsigned char * data = costmap_->getCharMap();
  memcpy(costmap_raw_->data.data(), data, costmap_raw_->data.size());
}

std::unique_ptr<map_msgs::msg::OccupancyGridUpdate> Costmap2DPublisher::createGridUpdateMsg()
{
  auto update = std::make_unique<map_msgs::msg::OccupancyGridUpdate>();

  update->header.stamp = clock_->now();
  update->header.frame_id = global_frame_;
  update->x = x0_;
  update->y = y0_;
  update->width = xn_ - x0_;
  update->height = yn_ - y0_;
  update->data.resize(update->width * update->height);

  std::uint32_t i = 0;
  for (std::uint32_t y = y0_; y < yn_; y++) {
    for (std::uint32_t x = x0_; x < xn_; x++) {
      update->data[i++] = cost_translation_table_[costmap_->getCost(x, y)];
    }
  }
  return update;
}

std::unique_ptr<nav2_msgs::msg::CostmapUpdate> Costmap2DPublisher::createCostmapUpdateMsg()
{
  auto msg = std::make_unique<nav2_msgs::msg::CostmapUpdate>();

  msg->header.stamp = clock_->now();
  msg->header.frame_id = global_frame_;
  msg->x = x0_;
  msg->y = y0_;
  msg->size_x = xn_ - x0_;
  msg->size_y = yn_ - y0_;
  msg->data.resize(msg->size_x * msg->size_y);

  std::uint32_t i = 0;
  for (std::uint32_t y = y0_; y < yn_; y++) {
    for (std::uint32_t x = x0_; x < xn_; x++) {
      msg->data[i++] = costmap_->getCost(x, y);
    }
  }
  return msg;
}

void Costmap2DPublisher::publishCostmap()
{
  float resolution = costmap_->getResolution();
  if (always_send_full_costmap_ || grid_resolution_ != resolution ||
    grid_width_ != costmap_->getSizeInCellsX() ||
    grid_height_ != costmap_->getSizeInCellsY() ||
    saved_origin_x_ != costmap_->getOriginX() ||
    saved_origin_y_ != costmap_->getOriginY())
  {
    updateGridParams();
    if (costmap_pub_->get_subscription_count() > 0) {
      prepareGrid();
      costmap_pub_->publish(std::move(grid_));
    }
    if (costmap_raw_pub_->get_subscription_count() > 0) {
      prepareCostmap();
      costmap_raw_pub_->publish(std::move(costmap_raw_));
    }
  } else if (x0_ < xn_) {
    // Publish just update msgs
    std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    if (costmap_update_pub_->get_subscription_count() > 0) {
      costmap_update_pub_->publish(createGridUpdateMsg());
    }
    if (costmap_raw_update_pub_->get_subscription_count() > 0) {
      costmap_raw_update_pub_->publish(createCostmapUpdateMsg());
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
  RCLCPP_DEBUG(logger_, "Received costmap service request");

  // TODO(bpwilcox): Grab correct orientation information
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.0);

  auto size_x = costmap_->getSizeInCellsX();
  auto size_y = costmap_->getSizeInCellsY();
  auto data_length = size_x * size_y;
  unsigned char * data = costmap_->getCharMap();
  auto current_time = clock_->now();

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
