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
    : last_min_x_(-std::numeric_limits<float>::max())
    , last_min_y_(-std::numeric_limits<float>::max())
    , last_max_x_(std::numeric_limits<float>::max())
    , last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void SemanticSegmentationLayer::onInitialize()
{
    auto node = node_.lock();
    if (!node)
    {
        throw std::runtime_error{"Failed to lock node"};
    }
    std::string segmentation_topic, camera_info_topic, pointcloud_topic;
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);
    declareParameter("segmentation_topic", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "segmentation_topic", segmentation_topic);
    declareParameter("camera_info_topic", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "camera_info_topic", camera_info_topic);
    declareParameter("pointcloud_topic", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "pointcloud_topic", pointcloud_topic);

    max_range_point_.point.x = 5.0;
    max_range_point_.header.frame_id = robot_base_frame_;

    semantic_segmentation_sub_ = node->create_subscription<vision_msgs::msg::SemanticSegmentation>(segmentation_topic, rclcpp::SensorDataQoS(), std::bind(&SemanticSegmentationLayer::segmentationCb, this, std::placeholders::_1));

    tracer_.initialize(rclcpp_node_, camera_info_topic, pointcloud_topic, false, tf_, "base_link", "camera", tf2::Duration(0), 10.0);

    need_recalculation_ = false;
    current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void SemanticSegmentationLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                             double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (need_recalculation_)
    {
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        // For some reason when I make these -<double>::max() it does not
        // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
        // -<float>::max() instead.
        *min_x = -std::numeric_limits<float>::max();
        *min_y = -std::numeric_limits<float>::max();
        *max_x = std::numeric_limits<float>::max();
        *max_y = std::numeric_limits<float>::max();
        need_recalculation_ = false;
    }
    else
    {
        double tmp_min_x = last_min_x_;
        double tmp_min_y = last_min_y_;
        double tmp_max_x = last_max_x_;
        double tmp_max_y = last_max_y_;
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        *min_x = std::min(tmp_min_x, *min_x);
        *min_y = std::min(tmp_min_y, *min_y);
        *max_x = std::max(tmp_max_x, *max_x);
        *max_y = std::max(tmp_max_y, *max_y);
    }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void SemanticSegmentationLayer::onFootprintChanged()
{
    need_recalculation_ = true;

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

    // master_array - is a direct pointer to the resulting master_grid.
    // master_grid - is a resulting costmap combined from all layers.
    // By using this pointer all layers will be overwritten!
    // To work with costmap layer and merge it with other costmap layers,
    // please use costmap_ pointer instead (this is pointer to current
    // costmap layer grid) and then call one of updates methods:
    // - updateWithAddition()
    // - updateWithMax()
    // - updateWithOverwrite()
    // - updateWithTrueOverwrite()
    // In this case using master_array pointer is equal to modifying local costmap_
    // pointer and then calling updateWithTrueOverwrite():
    unsigned char* master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

    // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
    // These variables are used to update the costmap only within this window
    // avoiding the updates of whole area.
    //
    // Fixing window coordinates with map size if necessary.
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);

    // Simply computing one-by-one cost per each cell
    int gradient_index;
    for (int j = min_j; j < max_j; j++)
    {
        // Reset gradient_index each time when reaching the end of re-calculated window
        // by OY axis.
        gradient_index = 0;
        for (int i = min_i; i < max_i; i++)
        {
            int index = master_grid.getIndex(i, j);
            // setting the gradient cost
            unsigned char cost = (LETHAL_OBSTACLE - gradient_index * GRADIENT_FACTOR) % 255;
            if (gradient_index <= GRADIENT_SIZE)
            {
                gradient_index++;
            }
            else
            {
                gradient_index = 0;
            }
            master_array[index] = cost;
        }
    }
}

void SemanticSegmentationLayer::segmentationCb(vision_msgs::msg::SemanticSegmentation::SharedPtr msg)
{
    latest_segmentation_message = *msg;
    // updateCostmap(latest_segmentation_message);
    geometry_msgs::msg::PointStamped max_range_point_cam_frame;
    if (!tf_->canTransform(
      latest_segmentation_message.header.frame_id, robot_base_frame_, tf2_ros::fromMsg(latest_segmentation_message.header.stamp)))
    {
        RCLCPP_INFO(
        logger_, "Range sensor layer can't transform from %s to %s",
        global_frame_.c_str(), latest_segmentation_message.header.frame_id.c_str());
        return;
    }
    tf_->transform(max_range_point_, max_range_point_cam_frame, latest_segmentation_message.header.frame_id, transform_tolerance_);
    tracer_.worldToImage(max_range_point_cam_frame)
}

void SemanticSegmentationLayer::updateCostmap(vision_msgs::msg::SemanticSegmentation& msg)
{
    cv::Mat mask_image(msg.width, msg.height, CV_8U);
    mask_image.setTo(msg.data);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::map<int, std::vector<geometry_msgs::msg::Point>> bird_view_polygons;
    cv::findContours(mask_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::map<int, std::string> class_map;
    for(auto& class_type : msg.class_map)
    {
        class_map[class_type.class_id] = class_type.class_name;
        // cv::Mat
    }
    // for(auto& contour : contours)
    // {

    // }
    for(unsigned i = 0; i < msg.data.size(); i++){
        if(msg.data[i] == 0)
        {
            continue;
        }
        else if(msg.data[i] == 1)
        {
            int mx, my;
            (void) mx;
            (void) my;
            // if(worldToMap(tracer_->Ray))
        }
    }
}

void SemanticSegmentationLayer::extractPolygons(vision_msgs::msg::SemanticSegmentation& msg)
{
    (void) msg;
}

}  // namespace nav2_costmap_2d

// This is the macro allowing a nav2_costmap_2d::SemanticSegmentationLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SemanticSegmentationLayer, nav2_costmap_2d::Layer)