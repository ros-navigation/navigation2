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

namespace nav2_costmap_2d
{

SemanticSegmentationLayer::SemanticSegmentationLayer() {}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void SemanticSegmentationLayer::onInitialize()
{
  current_ = true;
  was_reset_ = false;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  std::string segmentation_topic, pointcloud_topic, labels_topic, sensor_frame;
  std::vector<std::string> class_types_string;
  double max_obstacle_distance, min_obstacle_distance, observation_keep_time, transform_tolerance,
    expected_update_rate;
  bool track_unknown_space;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  declareParameter("publish_debug_topics", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("transform_tolerance", transform_tolerance);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();

  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  matchSize();

  node->get_parameter(name_ + "." + "observation_sources", topics_string_);

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string_);

  std::string source;

  while (ss >> source) {
    declareParameter(source + "." + "segmentation_topic", rclcpp::ParameterValue(""));
    declareParameter(source + "." + "labels_topic", rclcpp::ParameterValue(""));
    declareParameter(source + "." + "pointcloud_topic", rclcpp::ParameterValue(""));
    declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(""));
    declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
    declareParameter(
      source + "." + "class_types",
      rclcpp::ParameterValue(std::vector<std::string>({})));
    declareParameter(source + "." + "max_obstacle_distance", rclcpp::ParameterValue(5.0));
    declareParameter(source + "." + "min_obstacle_distance", rclcpp::ParameterValue(0.3));

    node->get_parameter(name_ + "." + source + "." + "segmentation_topic", segmentation_topic);
    node->get_parameter(name_ + "." + source + "." + "labels_topic", labels_topic);
    node->get_parameter(name_ + "." + source + "." + "pointcloud_topic", pointcloud_topic);
    node->get_parameter(
      name_ + "." + source + "." + "observation_persistence",
      observation_keep_time);
    node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
    node->get_parameter(name_ + "." + source + "." + "expected_update_rate", expected_update_rate);
    node->get_parameter(name_ + "." + source + "." + "class_types", class_types_string);
    node->get_parameter(
      name_ + "." + source + "." + "max_obstacle_distance",
      max_obstacle_distance);
    node->get_parameter(
      name_ + "." + source + "." + "min_obstacle_distance",
      min_obstacle_distance);
    if (class_types_string.empty()) {
      RCLCPP_ERROR(
        logger_,
        "no class types defined for source %s. Segmentation plugin cannot work this way",
        source.c_str());
      exit(-1);
    }

    std::unordered_map<std::string, uint8_t> class_map;

    for (auto & class_type : class_types_string) {
      std::vector<std::string> classes_ids;
      uint8_t cost;
      declareParameter(
        source + "." + class_type + ".classes",
        rclcpp::ParameterValue(std::vector<std::string>({})));
      declareParameter(source + "." + class_type + ".cost", rclcpp::ParameterValue(0));
      node->get_parameter(name_ + "." + source + "." + class_type + ".classes", classes_ids);
      if (classes_ids.empty()) {
        RCLCPP_ERROR(logger_, "no classes defined on type %s", class_type.c_str());
        continue;
      }
      node->get_parameter(name_ + "." + source + "." + class_type + ".cost", cost);
      for (auto & class_id : classes_ids) {
        class_map.insert(std::pair<std::string, uint8_t>(class_id, cost));
      }
    }

    if (class_map.empty()) {
      RCLCPP_ERROR(
        logger_,
        "No classes defined for source %s. Segmentation plugin cannot work this way",
        source.c_str());
      exit(-1);
    }

    //sensor data subscriptions
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;

    // label info subscription
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> tl_sub_opt;
    tl_sub_opt.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    tl_sub_opt.callback_group = callback_group_;
    rmw_qos_profile_t tl_qos = rmw_qos_profile_system_default;
    tl_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    tl_qos.depth = 5;
    tl_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    tl_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    auto segmentation_buffer = std::make_shared<nav2_costmap_2d::SegmentationBuffer>(
      node, source, class_types_string, class_map, observation_keep_time, expected_update_rate,
      max_obstacle_distance,
      min_obstacle_distance, *tf_, global_frame_, sensor_frame,
      tf2::durationFromSec(transform_tolerance));

    segmentation_buffers_.push_back(segmentation_buffer);


    auto semantic_segmentation_sub =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image,
        rclcpp_lifecycle::LifecycleNode>>(
      node, segmentation_topic, custom_qos_profile, sub_opt);
    semantic_segmentation_subs_.push_back(semantic_segmentation_sub);
    // semantic_segmentation_sub->registerCallback([&](std::shared_ptr<const sensor_msgs::msg::Image> /*msg*/){
    //   std::cout << "got sgm" << std::endl;
    // });

    auto label_info_sub = std::make_shared<message_filters::Subscriber<vision_msgs::msg::LabelInfo,
        rclcpp_lifecycle::LifecycleNode>>(
      node, labels_topic, tl_qos, tl_sub_opt);
    label_info_sub->registerCallback(
      std::bind(
        &SemanticSegmentationLayer::labelinfoCb, this,
        std::placeholders::_1, segmentation_buffers_.back()));
    label_info_subs_.push_back(label_info_sub);

    auto pointcloud_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
        rclcpp_lifecycle::LifecycleNode>>(
      node, pointcloud_topic, custom_qos_profile, sub_opt);
    pointcloud_subs_.push_back(pointcloud_sub);
    // pointcloud_sub->registerCallback([&](std::shared_ptr<const sensor_msgs::msg::PointCloud2> /*msg*/){
    //   std::cout << "got pc" << std::endl;
    // });

    auto pointcloud_tf_sub =
      std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      *pointcloud_subs_.back(), *tf_, global_frame_, 1000, node->get_node_logging_interface(),
      node->get_node_clock_interface(),
      tf2::durationFromSec(transform_tolerance));
    // pointcloud_tf_sub->registerCallback([&](std::shared_ptr<const sensor_msgs::msg::PointCloud2> /*msg*/){
    //   std::cout << "got pc tf" << std::endl;
    // });
    pointcloud_tf_subs_.push_back(pointcloud_tf_sub);

    auto segm_pc_sync =
      std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
        sensor_msgs::msg::PointCloud2>>(
      *semantic_segmentation_subs_.back(), *pointcloud_tf_subs_.back(), 1000);
    segm_pc_sync->registerCallback(
      std::bind(
        &SemanticSegmentationLayer::syncSegmPointcloudCb, this,
        std::placeholders::_1, std::placeholders::_2, segmentation_buffers_.back()));

    segm_pc_notifiers_.push_back(segm_pc_sync);
  }

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &SemanticSegmentationLayer::dynamicParametersCallback,
      this,
      std::placeholders::_1));
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void SemanticSegmentationLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  std::vector<nav2_costmap_2d::Segmentation> segmentations;
  getSegmentations(segmentations);


  current_ = true;

  for (auto & segmentation : segmentations) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*segmentation.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*segmentation.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_class(*segmentation.cloud_, "class");
    // sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_confidence(*segmentation.cloud_,
    // "confidence");
    for (size_t point = 0; point < segmentation.cloud_->height * segmentation.cloud_->width;
      point++)
    {
      unsigned int mx, my;
      if (!worldToMap(*(iter_x + point), *(iter_y + point), mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }
      unsigned int index = getIndex(mx, my);
      uint8_t class_id = *(iter_class + point);
      if (!segmentation.class_map_.count(class_id)) {
        RCLCPP_DEBUG(logger_, "Cost for class id %i was not defined, skipping", class_id);
        continue;
      }
      costmap_[index] = segmentation.class_map_[class_id];
      touch(*(iter_x + point), *(iter_y + point), min_x, min_y, max_x, max_y);
    }
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void SemanticSegmentationLayer::onFootprintChanged()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("nav2_costmap_2d"),
    "SemanticSegmentationLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void SemanticSegmentationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i,
  int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
  if (!costmap_) {
    return;
  }
  // RCLCPP_INFO(logger_, "Updating costmap");
  switch (combination_method_) {
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

void SemanticSegmentationLayer::labelinfoCb(
  const std::shared_ptr<const vision_msgs::msg::LabelInfo> & label_info,
  const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer> & buffer)
{
  buffer->createClassIdCostMap(*label_info);
}

void SemanticSegmentationLayer::syncSegmPointcloudCb(
  const std::shared_ptr<const sensor_msgs::msg::Image> & segmentation,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & pointcloud,
  const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer> & buffer)
{
  if (segmentation->width * segmentation->height != pointcloud->width * pointcloud->height) {
    RCLCPP_WARN(
      logger_,
      "Pointcloud and segmentation sizes are different, will not buffer message. "
      "segmentation->width:%u,  "
      "segmentation->height:%u, pointcloud->width:%u, pointcloud->height:%u",
      segmentation->width, segmentation->height, pointcloud->width, pointcloud->height);
    return;
  }
  unsigned expected_array_size = segmentation->width * segmentation->height;
  if (segmentation->data.size() < expected_array_size) {
    RCLCPP_WARN(
      logger_,
      "segmentation arrays have wrong sizes: data->%lu, expected->%u. "
      "Will not buffer message",
      segmentation->data.size(), expected_array_size);
    return;
  }
  if (buffer->isClassIdCostMapEmpty()) {
    RCLCPP_WARN(
      logger_,
      "Class map is empty because a labelinfo message has not been received for topic %s. Will not buffer message",
      buffer->getBufferSource().c_str());
    return;
  }
  buffer->lock();
  // we are passing the segmentation as confidence temporarily while we figure out a good use for getting
  // confidence maps
  buffer->bufferSegmentation(*pointcloud, *segmentation, *segmentation);
  buffer->unlock();
}

void SemanticSegmentationLayer::reset()
{
  resetMaps();
  current_ = false;
  was_reset_ = true;
}

bool SemanticSegmentationLayer::getSegmentations(
  std::vector<nav2_costmap_2d::Segmentation> & segmentations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < segmentation_buffers_.size(); ++i) {
    segmentation_buffers_[i]->lock();
    segmentation_buffers_[i]->getSegmentations(segmentations);
    current = segmentation_buffers_[i]->isCurrent() && current;
    segmentation_buffers_[i]->unlock();
  }
  return current;
}

rcl_interfaces::msg::SetParametersResult
SemanticSegmentationLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  auto result = rcl_interfaces::msg::SetParametersResult();
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }

    std::stringstream ss(topics_string_);
    std::string source;
    while (ss >> source) {
      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (name == name_ + "." + source + "." + "max_obstacle_distance") {
          for (auto & buffer : segmentation_buffers_) {
            if (buffer->getBufferSource() == source) {
              buffer->setMaxObstacleDistance(parameter.as_double());
            }
          }
        } else if (name == name_ + "." + source + "." + "min_obstacle_distance") {
          for (auto & buffer : segmentation_buffers_) {
            if (buffer->getBufferSource() == source) {
              buffer->setMinObstacleDistance(parameter.as_double());
            }
          }
        }
        // allow to change which class ids belong to each type
      } else if (type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
        for (auto & buffer : segmentation_buffers_) {
          if (buffer->getBufferSource() == source) {
            for (auto & class_type : buffer->getClassTypes()) {
              if (name == name_ + "." + source + "." + class_type + "." + "classes") {
                int class_type_cost;
                node_.lock()->get_parameter(
                  name_ + "." + source + "." + class_type + ".cost",
                  class_type_cost);
                for (auto & class_name : parameter.as_string_array()) {
                  buffer->updateClassMap(class_name, class_type_cost);
                }
              }
            }
          }
        }
      } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        for (auto & buffer : segmentation_buffers_) {
          if (buffer->getBufferSource() == source) {
            for (auto & class_type : buffer->getClassTypes()) {
              if (name == name_ + "." + source + "." + class_type + "." + "cost") {
                std::vector<std::string> class_names_for_type;
                node_.lock()->get_parameter(
                  name_ + "." + source + "." + class_type + ".classes",
                  class_names_for_type);
                for (auto & class_name : class_names_for_type) {
                  buffer->updateClassMap(class_name, parameter.as_int());
                }
              }
            }
          }
        }
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d

// This is the macro allowing a nav2_costmap_2d::SemanticSegmentationLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SemanticSegmentationLayer, nav2_costmap_2d::Layer)
