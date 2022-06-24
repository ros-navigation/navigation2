/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
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
 * Author: Pedro Gonzalez
 *********************************************************************/
#include "nav2_costmap_2d/semantic_segmentation_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_costmap_2d {

SemanticSegmentationLayer::SemanticSegmentationLayer() {}

void SemanticSegmentationLayer::onInitialize()
{
  current_ = true;
  was_reset_ = false;
  auto node = node_.lock();
  if (!node)
  {
    throw std::runtime_error{"Failed to lock node"};
  }
  std::string segmentation_topic, pointcloud_topic, sensor_frame;
  std::vector<std::string> class_types_string;
  double max_obstacle_distance, min_obstacle_distance, observation_keep_time, transform_tolerance,
    expected_update_rate;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("publish_debug_topics", rclcpp::ParameterValue(false));
  declareParameter("max_obstacle_distance", rclcpp::ParameterValue(5.0));
  declareParameter("min_obstacle_distance", rclcpp::ParameterValue(0.3));
  declareParameter("segmentation_topic", rclcpp::ParameterValue(""));
  declareParameter("pointcloud_topic", rclcpp::ParameterValue(""));
  declareParameter("observation_persistence", rclcpp::ParameterValue(0.0));
  declareParameter("expected_update_rate", rclcpp::ParameterValue(0.0));
  declareParameter("class_types", rclcpp::ParameterValue(std::vector<std::string>({})));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter(name_ + "." + "publish_debug_topics", debug_topics_);
  node->get_parameter(name_ + "." + "max_obstacle_distance", max_obstacle_distance);
  node->get_parameter(name_ + "." + "min_obstacle_distance", min_obstacle_distance);
  node->get_parameter(name_ + "." + "segmentation_topic", segmentation_topic);
  node->get_parameter(name_ + "." + "pointcloud_topic", pointcloud_topic);
  node->get_parameter(name_ + "." + "observation_persistence", observation_keep_time);
  node->get_parameter(name_ + "." + "sensor_frame", sensor_frame);
  node->get_parameter(name_ + "." + "expected_update_rate", expected_update_rate);
  node->get_parameter("transform_tolerance", transform_tolerance);
  node->get_parameter(name_ + "." + "class_types", class_types_string);
  if (class_types_string.empty())
  {
    RCLCPP_ERROR(logger_, "no class types defined. Segmentation plugin cannot work this way");
    exit(-1);
  }

  for (auto& source : class_types_string)
  {
    // get the cost for each class
    std::vector<std::string> classes_ids;
    uint8_t cost;
    declareParameter(source + ".classes", rclcpp::ParameterValue(std::vector<std::string>({})));
    declareParameter(source + ".cost", rclcpp::ParameterValue(0));
    node->get_parameter(name_ + "." + source + ".classes", classes_ids);
    if (classes_ids.empty())
    {
      RCLCPP_ERROR(logger_, "no classes defined on type %s", source.c_str());
      continue;
    }
    node->get_parameter(name_ + "." + source + ".cost", cost);
    for (auto& class_id : classes_ids)
    {
      class_map_.insert(std::pair<std::string, uint8_t>(class_id, cost));
    }
  }

  if (class_map_.empty())
  {
    RCLCPP_ERROR(logger_, "No classes defined. Segmentation plugin cannot work this way");
    exit(-1);
  }

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();

  matchSize();

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;

  semantic_segmentation_sub_ =
    std::make_shared<message_filters::Subscriber<vision_msgs::msg::SemanticSegmentation>>(
      rclcpp_node_, segmentation_topic, custom_qos_profile);
  pointcloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
    rclcpp_node_, pointcloud_topic, custom_qos_profile);
  pointcloud_tf_sub_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    *pointcloud_sub_, *tf_, global_frame_, 50, rclcpp_node_,
    tf2::durationFromSec(transform_tolerance));
  // make sure all data is synced: pointclouds, transforms and segmentations
  segm_pc_sync_ =
    std::make_shared<message_filters::TimeSynchronizer<vision_msgs::msg::SemanticSegmentation,
                                                       sensor_msgs::msg::PointCloud2>>(
      *semantic_segmentation_sub_, *pointcloud_tf_sub_, 100);
  segm_pc_sync_->registerCallback(std::bind(&SemanticSegmentationLayer::syncSegmPointcloudCb, this,
                                            std::placeholders::_1, std::placeholders::_2));

  // create a segmentation buffer
  segmentation_buffer_ = std::make_shared<nav2_costmap_2d::SegmentationBuffer>(
    node, pointcloud_topic, observation_keep_time, expected_update_rate, max_obstacle_distance,
    min_obstacle_distance, *tf_, global_frame_, sensor_frame,
    tf2::durationFromSec(transform_tolerance));

  // only for testing purposes
  if (debug_topics_)
  {
    sgm_debug_pub_ =
      node->create_publisher<vision_msgs::msg::SemanticSegmentation>("/buffered_segmentation", 1);
    orig_pointcloud_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/buffered_pointcloud", 1);
    proc_pointcloud_pub_ =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_pointcloud", 1);
  }
}


void SemanticSegmentationLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                                             double* min_x, double* min_y, double* max_x,
                                             double* max_y)
{
  if (rolling_window_)
  {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_)
  {
    current_ = true;
    return;
  }
  std::vector<nav2_costmap_2d::Segmentation> segmentations;
  // get the latest segmentations thread safely
  segmentation_buffer_->lock();
  segmentation_buffer_->getSegmentations(segmentations);
  segmentation_buffer_->unlock();
  for (auto& segmentation : segmentations)
  {
    if (debug_topics_)
    {
      proc_pointcloud_pub_->publish(*segmentation.cloud_);
    }
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*segmentation.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*segmentation.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_class(*segmentation.cloud_, "class");
    // For now, confidence of the inference is not used to adjust cost
    // sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_confidence(*segmentation.cloud_,
    // "confidence");
    for (size_t point = 0; point < segmentation.cloud_->height * segmentation.cloud_->width;
         point++)
    {
      unsigned int mx, my;
      if (!worldToMap(*(iter_x + point), *(iter_y + point), mx, my))
      {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }
      unsigned int index = getIndex(mx, my);
      uint8_t class_id = *(iter_class + point);
      // Check if no cost was defined for the given class
      if (!class_map_.count(segmentation.class_map_[class_id]))
      {
        RCLCPP_DEBUG(logger_, "Cost for class id %i was not defined, skipping", class_id);
        continue;
      }
      // Get the class name from the buffered segmentation and then gets its cost from the class map
      costmap_[index] = class_map_[segmentation.class_map_[class_id]];
      touch(*(iter_x + point), *(iter_y + point), min_x, min_y, max_x, max_y);
    }
  }
}

void SemanticSegmentationLayer::onFootprintChanged()
{
  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"),
               "SemanticSegmentationLayer::onFootprintChanged(): num footprint points: %lu",
               layered_costmap_->getFootprint().size());
}

void SemanticSegmentationLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i,
                                            int min_j, int max_i, int max_j)
{
  if (!enabled_)
  {
    return;
  }

  if (!current_ && was_reset_)
  {
    was_reset_ = false;
    current_ = true;
  }
  if (!costmap_)
  {
    return;
  }
  switch (combination_method_)
  {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void SemanticSegmentationLayer::syncSegmPointcloudCb(
  const std::shared_ptr<const vision_msgs::msg::SemanticSegmentation>& segmentation,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& pointcloud)
{
  // check if data has the right dimensions
  if (segmentation->width * segmentation->height != pointcloud->width * pointcloud->height)
  {
    RCLCPP_WARN(logger_,
                "Pointcloud and segmentation sizes are different, will not buffer message. "
                "segmentation->width:%u,  "
                "segmentation->height:%u, pointcloud->width:%u, pointcloud->height:%u",
                segmentation->width, segmentation->height, pointcloud->width, pointcloud->height);
    return;
  }
  unsigned expected_array_size = segmentation->width * segmentation->height;
  if (segmentation->data.size() < expected_array_size ||
      segmentation->confidence.size() < expected_array_size)
  {
    RCLCPP_WARN(logger_,
                "segmentation arrays have wrong sizes: data->%lu, confidence->%lu, expected->%u. "
                "Will not buffer message",
                segmentation->data.size(), segmentation->confidence.size(), expected_array_size);
    return;
  }
  // check if class names and ids correspondence is contained in the message
  if (segmentation->class_map.size() == 0)
  {
    RCLCPP_WARN(logger_, "Classs map is empty. Will not buffer message");
    return;
  }
  // Store the pointcloud and segmentation
  segmentation_buffer_->lock();
  segmentation_buffer_->bufferSegmentation(*pointcloud, *segmentation);
  segmentation_buffer_->unlock();
  if (debug_topics_)
  {
    sgm_debug_pub_->publish(*segmentation);
    orig_pointcloud_pub_->publish(*pointcloud);
  }
}

void SemanticSegmentationLayer::reset()
{
  resetMaps();
  current_ = false;
  was_reset_ = true;
}

}  // namespace nav2_costmap_2d
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SemanticSegmentationLayer, nav2_costmap_2d::Layer)