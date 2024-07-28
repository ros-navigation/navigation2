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
#ifndef SEMANTIC_SEGMENTATION_LAYER_HPP_
#define SEMANTIC_SEGMENTATION_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/segmentation.hpp"
#include "nav2_costmap_2d/segmentation_buffer.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/message_filter.h"
#include "vision_msgs/msg/label_info.hpp"

namespace nav2_costmap_2d
{

/**
 * @class SemanticSegmentationLayer
 * @brief Takes in semantic segmentation messages and aligned pointclouds to populate the 2D costmap
 */
class SemanticSegmentationLayer : public CostmapLayer
{
public:
  /**
   * @brief A constructor
   */
  SemanticSegmentationLayer();

  /**
   * @brief A destructor
   */
  virtual ~SemanticSegmentationLayer() {}

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();
  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);
  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
    int max_i, int max_j);

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  virtual void onFootprintChanged();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() {return true;}

  bool getSegmentations(std::vector<nav2_costmap_2d::Segmentation> & segmentations) const;

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

private:
  void syncSegmPointcloudCb(
    const std::shared_ptr<const sensor_msgs::msg::Image> & segmentation,
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & pointcloud,
    const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer> & buffer);

  void labelinfoCb(
    const std::shared_ptr<const vision_msgs::msg::LabelInfo> & label_info,
    const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer> & buffer);

  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image,
    rclcpp_lifecycle::LifecycleNode>>>
  semantic_segmentation_subs_;
  std::vector<
    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::LabelInfo,
    rclcpp_lifecycle::LifecycleNode>>>
  label_info_subs_;
  std::vector<
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
    rclcpp_lifecycle::LifecycleNode>>>
  pointcloud_subs_;
  std::vector<
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
    sensor_msgs::msg::PointCloud2>>>
  segm_pc_notifiers_;
  std::vector<std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>>
  pointcloud_tf_subs_;

  // debug publishers
  std::map<std::string,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>> proc_pointcloud_pubs_map_;

  std::vector<std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>> segmentation_buffers_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  std::string global_frame_;
  std::string topics_string_;

  std::map<std::string, uint8_t> class_map_;

  bool rolling_window_;
  bool was_reset_;
  int combination_method_;
};

}  // namespace nav2_costmap_2d

#endif  // SEMANTIC_SEGMENTATION_LAYER_HPP_
