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

SemanticSegmentationLayer::SemanticSegmentationLayer()
{
}

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
    std::string segmentation_topic, camera_info_topic, pointcloud_topic;
    bool use_pointcloud;
    double max_lookahead_distance;

    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);
    declareParameter("use_pointcloud", rclcpp::ParameterValue(false));
    node->get_parameter(name_ + "." + "use_pointcloud", use_pointcloud);
    declareParameter("max_lookahead_distance", rclcpp::ParameterValue(3.0));
    node->get_parameter(name_ + "." + "max_lookahead_distance", max_lookahead_distance);
    declareParameter("segmentation_topic", rclcpp::ParameterValue(""));
    node->get_parameter(name_ + "." + "segmentation_topic", segmentation_topic);
    declareParameter("camera_info_topic", rclcpp::ParameterValue(""));
    node->get_parameter(name_ + "." + "camera_info_topic", camera_info_topic);
    declareParameter("pointcloud_topic", rclcpp::ParameterValue(""));
    node->get_parameter(name_ + "." + "pointcloud_topic", pointcloud_topic);

    global_frame_ = layered_costmap_->getGlobalFrameID();
    rolling_window_ = layered_costmap_->isRolling();

    matchSize();

    semantic_segmentation_sub_ = node->create_subscription<vision_msgs::msg::SemanticSegmentation>(segmentation_topic, rclcpp::SensorDataQoS(), std::bind(&SemanticSegmentationLayer::segmentationCb, this, std::placeholders::_1));

    tracer_.initialize(rclcpp_node_, camera_info_topic, pointcloud_topic, use_pointcloud, tf_, global_frame_, tf2::Duration(0), max_lookahead_distance);

}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void SemanticSegmentationLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                                             double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (rolling_window_) {
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }
    if (!enabled_) {
        current_ = true;
        return;
    } 
    vision_msgs::msg::SemanticSegmentation current_segmentation = latest_segmentation_message;
    int touched_cells = 0;
    for(size_t v = 0; v < current_segmentation.height; v=v+5)
    {
        for(size_t u = 0; u < current_segmentation.width; u=u+5){
            uint8_t pixel = current_segmentation.data[v*current_segmentation.width+u];
            cv::Point2d pixel_idx;
            pixel_idx.x = u;
            pixel_idx.y = v;
            geometry_msgs::msg::PointStamped world_point;
            if(!tracer_.imageToGroundPlane(pixel_idx, world_point))
            {
                RCLCPP_DEBUG(logger_, "Could not raycast");
                continue;
            }
            unsigned int mx, my;
            if(!worldToMap(world_point.point.x, world_point.point.y, mx, my))
            {
                RCLCPP_DEBUG(logger_, "Computing map coords failed");
                continue;
            }
            unsigned int index = getIndex(mx, my);
            if(pixel!= 1){
                costmap_[index] = 100;
            }else{
                costmap_[index] = 0;
            }
            touch(world_point.point.x, world_point.point.y, min_x, min_y, max_x, max_y);
            touched_cells++;
        }
    }
    // RCLCPP_INFO(logger_, "touched %i cells", touched_cells);
    
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
void SemanticSegmentationLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                            int max_j)
{
    if (!enabled_)
    {
        return;
    }

    if (!current_ && was_reset_) {
        was_reset_ = false;
        current_ = true;
    }
    // (void) max_j;
    // (void) master_grid;
    // (void) min_i;
    // (void) max_i;
    // (void) min_j;
    if(!costmap_)
    {
        return;
    }
    // RCLCPP_INFO(logger_, "Updating costmap");
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

void SemanticSegmentationLayer::segmentationCb(vision_msgs::msg::SemanticSegmentation::SharedPtr msg)
{
    latest_segmentation_message = *msg;
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