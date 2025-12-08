// Copyright (c) 2025 Berkan Tali
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_COSTMAP_2D__TRACKING_BOUNDS_LAYER_HPP_
#define NAV2_COSTMAP_2D__TRACKING_BOUNDS_LAYER_HPP_

#include <vector>
#include <memory>
#include <mutex>

#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/tracking_feedback.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_util/path_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace nav2_costmap_2d
{
class TrackingBoundsLayer : public nav2_costmap_2d::Layer
{
public:
  TrackingBoundsLayer() = default;

  ~TrackingBoundsLayer();
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  virtual void reset();
  virtual bool isClearable() {return false;}

  // Lifecycle methods
  virtual void activate();
  virtual void deactivate();
  std::vector<std::vector<double>> getWallPoints(const nav_msgs::msg::Path & segment);
  nav_msgs::msg::Path getPathSegment();

protected:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void trackingCallback(const nav2_msgs::msg::TrackingFeedback::SharedPtr msg);
  nav2_msgs::msg::TrackingFeedback last_tracking_feedback_;
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

private:
  nav2::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav2::Subscription<nav2_msgs::msg::TrackingFeedback>::SharedPtr tracking_feedback_sub_;
  std::mutex data_mutex_;
  nav_msgs::msg::Path last_path_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  size_t temp_step_;
  int step_;
  double width_;
  double look_ahead_;
  bool enabled_;
  std::string path_topic_;
  std::string tracking_feedback_topic_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__TRACKING_Bounds_LAYER_HPP_
