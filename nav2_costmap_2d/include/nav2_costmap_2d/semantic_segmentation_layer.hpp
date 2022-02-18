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
#ifndef SEMANTIC_SEGMENTATION_LAYER_HPP_
#define SEMANTIC_SEGMENTATION_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/ray_tracer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/semantic_segmentation.hpp"
#include "opencv2/core.hpp"

namespace nav2_costmap_2d {

class SemanticSegmentationLayer : public nav2_costmap_2d::Layer
{
   public:
    SemanticSegmentationLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y);
    virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void reset() { return; }

    virtual void onFootprintChanged();

    virtual bool isClearable() { return false; }

   private:
    void segmentationCb(vision_msgs::msg::SemanticSegmentation::SharedPtr msg);

    void updateCostmap(vision_msgs::msg::SemanticSegmentation& msg);

    void extractPolygons(vision_msgs::msg::SemanticSegmentation& msg);

    RayTracer tracer_;
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
    rclcpp::Subscription<vision_msgs::msg::SemanticSegmentation>::SharedPtr semantic_segmentation_sub_;

    vision_msgs::msg::SemanticSegmentation latest_segmentation_message;
    geometry_msgs::msg::TransformStamped transform_at_message;
    geometry_msgs::msg::PointStamped max_range_point_;

    std::string global_frame_;
    std::string robot_base_frame_;
    tf2::Duration transform_tolerance_;

    // Indicates that the entire gradient should be recalculated next time.
    bool need_recalculation_;

    // Size of gradient in cells
    int GRADIENT_SIZE = 20;
    // Step of increasing cost per one cell in gradient
    int GRADIENT_FACTOR = 10;
};

}  // namespace nav2_costmap_2d

#endif  // SEMANTIC_SEGMENTATION_LAYER_HPP_