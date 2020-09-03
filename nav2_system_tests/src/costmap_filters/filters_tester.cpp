// Copyright (c) 2020 Samsung Research Russia
// Copyright (c) 2018 Intel Corporation
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
// limitations under the License. Reserved.

#include "filters_tester.hpp"

#include <string>
#include <vector>
#include <memory>
#include <iomanip>

using namespace std::chrono_literals;

namespace nav2_system_tests
{

FiltersTester::FiltersTester()
: nav2_util::LifecycleNode("filters_tester"), is_active_(false)
{
  RCLCPP_INFO(get_logger(), "Creating");
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap_2d_ros", "/", "");
  costmap_ros_->set_parameter(rclcpp::Parameter("always_send_full_costmap", true));
  costmap_ros_->set_parameter(rclcpp::Parameter("resolution", 0.1));
  costmap_ros_->set_parameter(rclcpp::Parameter("publish_frequency", 5.0));
  costmap_ros_->set_parameter(rclcpp::Parameter("update_frequency", 5.0));
  costmap_ros_->set_parameter(rclcpp::Parameter("robot_radius", 0.2));

  std::vector<std::string> plugins_list{"static_layer", "obstacle_layer", "keepout_layer"};
  costmap_ros_->set_parameter(rclcpp::Parameter("plugins", plugins_list));
  // Since plugins are not initialized yet, plugins' parameters are not declared as well.
  // We need to declare them before setting.
  costmap_ros_->declare_parameter(
    "static_layer.plugin", rclcpp::ParameterValue("nav2_costmap_2d::StaticLayer"));
  costmap_ros_->set_parameter(
    rclcpp::Parameter(
      "static_layer.plugin", "nav2_costmap_2d::StaticLayer"));
  costmap_ros_->declare_parameter(
    "obstacle_layer.plugin", rclcpp::ParameterValue("nav2_costmap_2d::ObstacleLayer"));
  costmap_ros_->set_parameter(
    rclcpp::Parameter(
      "obstacle_layer.plugin", "nav2_costmap_2d::ObstacleLayer"));
  costmap_ros_->declare_parameter(
    "keepout_layer.plugin", rclcpp::ParameterValue("nav2_costmap_2d::KeepoutFilter"));
  costmap_ros_->set_parameter(
    rclcpp::Parameter(
      "keepout_layer.plugin", "nav2_costmap_2d::KeepoutFilter"));
  costmap_ros_->declare_parameter(
    "keepout_layer.filter_info_topic", rclcpp::ParameterValue("costmap_filter_info"));
  costmap_ros_->set_parameter(
    rclcpp::Parameter("keepout_layer.filter_info_topic", "costmap_filter_info"));

  planner_ = std::make_shared<nav2_navfn_planner::NavfnPlanner>();
}

FiltersTester::~FiltersTester()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
FiltersTester::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->on_configure(state);

  auto node = shared_from_this();
  auto tf = costmap_ros_->getTfBuffer();
  planner_->configure(node, "test_planner", tf, costmap_ros_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FiltersTester::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  startRobotTransform();

  costmap_ros_->on_activate(state);
  planner_->activate();

  is_active_ = true;

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FiltersTester::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  is_active_ = false;

  transform_timer_.reset();
  tf_broadcaster_.reset();

  costmap_ros_->on_deactivate(state);
  planner_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FiltersTester::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  costmap_ros_->on_cleanup(state);
  planner_->cleanup();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FiltersTester::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FiltersTester::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  costmap_ros_->on_shutdown(state);

  return nav2_util::CallbackReturn::SUCCESS;
}

void FiltersTester::startRobotTransform()
{
  // Provide the robot pose transform
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Set an initial pose
  geometry_msgs::msg::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  updateRobotPosition(robot_position);

  // Publish the transform periodically
  transform_timer_ = create_wall_timer(
    100ms, std::bind(&FiltersTester::publishRobotTransform, this));
}

void FiltersTester::updateRobotPosition(const geometry_msgs::msg::Point & position)
{
  if (!base_transform_) {
    base_transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
    base_transform_->header.frame_id = "map";
    base_transform_->child_frame_id = "base_link";
  }

  base_transform_->header.stamp = now() + rclcpp::Duration(250000000);
  base_transform_->transform.translation.x = position.x;
  base_transform_->transform.translation.y = position.y;
  base_transform_->transform.rotation.w = 1.0;

  publishRobotTransform();
}

void FiltersTester::publishRobotTransform()
{
  if (base_transform_) {
    tf_broadcaster_->sendTransform(*base_transform_);
  }
}

void FiltersTester::spinTester()
{
  if (rclcpp::ok()) {
    // Spin LifycycleNode and Costmap2DROS
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::spin_some(costmap_ros_->get_node_base_interface());
  }
}

bool FiltersTester::testPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end)
{
  geometry_msgs::msg::PoseStamped pose = start;
  nav_msgs::msg::Path path;
  bool result;

  do {
    spinTester();

    result = checkPlan(pose, end, path);
    if (result) {
      if (path.poses.size() > 2) {
        // Goal is not reached, continue moving
        // printPath(path);
        pose = path.poses[1];
        updateRobotPosition(pose.pose.position);
      } else {
        // Goal was sucessfully reached
        return true;
      }
      // Sleep for some time to let costmap to be updated
      std::this_thread::sleep_for(100ms);
    }
  } while (result);

  // Fail case: can not produce the path to the goal
  return false;
}

bool FiltersTester::checkPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  nav_msgs::msg::Path & path) const
{
  if (!is_active_) {
    RCLCPP_WARN(get_logger(), "FiltersTester node is not activated yet");
    return false;
  }

  try {
    path = planner_->createPlan(start, end);
    if (!path.poses.size()) {
      return false;
    }
  } catch (...) {
    return false;
  }
  return true;
}

void FiltersTester::printPath(const nav_msgs::msg::Path & path) const
{
  auto index = 0;
  auto ss = std::stringstream{};

  ss << '\n';
  for (auto pose : path.poses) {
    ss << "   point #" << index << " with" <<
      " x: " << std::setprecision(3) << pose.pose.position.x <<
      " y: " << std::setprecision(3) << pose.pose.position.y << '\n';
    ++index;
  }

  RCLCPP_INFO(get_logger(), ss.str().c_str());
}

}  // namespace nav2_system_tests
