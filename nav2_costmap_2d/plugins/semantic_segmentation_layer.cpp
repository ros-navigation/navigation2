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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
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

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
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
  declareParameter(name_ + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
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
  segm_pc_sync_ =
    std::make_shared<message_filters::TimeSynchronizer<vision_msgs::msg::SemanticSegmentation,
                                                       sensor_msgs::msg::PointCloud2>>(
      *semantic_segmentation_sub_, *pointcloud_tf_sub_, 100);
  segm_pc_sync_->registerCallback(std::bind(&SemanticSegmentationLayer::syncSegmPointcloudCb, this,
                                            std::placeholders::_1, std::placeholders::_2));

  segmentation_buffer_ = std::make_shared<nav2_costmap_2d::SegmentationBuffer>(
    node, pointcloud_topic, observation_keep_time, expected_update_rate, max_obstacle_distance,
    min_obstacle_distance, *tf_, global_frame_, sensor_frame,
    tf2::durationFromSec(transform_tolerance));

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

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
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
      if (!class_map_.count(segmentation.class_map_[class_id]))
      {
        RCLCPP_DEBUG(logger_, "Cost for class id %i was not defined, skipping", class_id);
        continue;
      }
      costmap_[index] = class_map_[segmentation.class_map_[class_id]];
      touch(*(iter_x + point), *(iter_y + point), min_x, min_y, max_x, max_y);
    }
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void SemanticSegmentationLayer::onFootprintChanged()
{
  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"),
               "SemanticSegmentationLayer::onFootprintChanged(): num footprint points: %lu",
               layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
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
  // RCLCPP_INFO(logger_, "Updating costmap");
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
  if (segmentation->class_map.size() == 0)
  {
    RCLCPP_WARN(logger_, "Classs map is empty. Will not buffer message");
    return;
  }
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

// This is the macro allowing a nav2_costmap_2d::SemanticSegmentationLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SemanticSegmentationLayer, nav2_costmap_2d::Layer)