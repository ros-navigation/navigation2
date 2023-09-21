// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/controller.hpp"

#include "models.hpp"

namespace detail
{

template<typename TMessage, typename TNode>
void setHeader(TMessage && msg, TNode node, std::string frame)
{
  auto time = node->get_clock()->now();
  msg.header.frame_id = frame;
  msg.header.stamp = time;
}

}  // namespace detail


/**
 * Adds some parameters for the optimizer to a special container.
 *
 * @param params_ container for optimizer's parameters.
 */
void setUpOptimizerParams(
  const TestOptimizerSettings & s,
  const std::vector<std::string> & critics,
  std::vector<rclcpp::Parameter> & params_, std::string node_name = std::string("dummy"))
{
  constexpr double dummy_freq = 50.0;
  params_.emplace_back(rclcpp::Parameter(node_name + ".iteration_count", s.iteration_count));
  params_.emplace_back(rclcpp::Parameter(node_name + ".batch_size", s.batch_size));
  params_.emplace_back(rclcpp::Parameter(node_name + ".time_steps", s.time_steps));
  params_.emplace_back(rclcpp::Parameter(node_name + ".lookahead_dist", s.lookahead_distance));
  params_.emplace_back(rclcpp::Parameter(node_name + ".motion_model", s.motion_model));
  params_.emplace_back(rclcpp::Parameter(node_name + ".critics", critics));
  params_.emplace_back(rclcpp::Parameter("controller_frequency", dummy_freq));
}

void setUpControllerParams(
  bool visualize, std::vector<rclcpp::Parameter> & params_,
  std::string node_name = std::string("dummy"))
{
  double dummy_freq = 50.0;
  params_.emplace_back(rclcpp::Parameter(node_name + ".visualize", visualize));
  params_.emplace_back(rclcpp::Parameter("controller_frequency", dummy_freq));
}

rclcpp::NodeOptions getOptimizerOptions(
  TestOptimizerSettings s,
  const std::vector<std::string> & critics)
{
  std::vector<rclcpp::Parameter> params;
  rclcpp::NodeOptions options;
  setUpOptimizerParams(s, critics, params);
  options.parameter_overrides(params);
  return options;
}

geometry_msgs::msg::Point getDummyPoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0;

  return point;
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getDummyCostmapRos()
{
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  costmap_ros->on_configure(rclcpp_lifecycle::State{});
  return costmap_ros;
}

std::shared_ptr<nav2_costmap_2d::Costmap2D> getDummyCostmap(TestCostmapSettings s)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    s.cells_x, s.cells_y, s.resolution, s.origin_x, s.origin_y, s.cost_map_default_value);

  return costmap;
}

std::vector<geometry_msgs::msg::Point> getDummySquareFootprint(double a)
{
  return {getDummyPoint(a, a), getDummyPoint(-a, -a), getDummyPoint(a, -a), getDummyPoint(-a, a)};
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getDummyCostmapRos(TestCostmapSettings s)
{
  auto costmap_ros = getDummyCostmapRos();
  auto costmap_ptr = costmap_ros->getCostmap();
  auto costmap = getDummyCostmap(s);
  *(costmap_ptr) = *costmap;

  costmap_ros->setRobotFootprint(getDummySquareFootprint(s.footprint_size));

  return costmap_ros;
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
getDummyNode(
  TestOptimizerSettings s, std::vector<std::string> critics,
  std::string node_name = std::string("dummy"))
{
  auto node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, getOptimizerOptions(s, critics));
  return node;
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
getDummyNode(rclcpp::NodeOptions options, std::string node_name = std::string("dummy"))
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  return node;
}

template<typename TNode, typename TCostMap, typename TParamHandler>
std::shared_ptr<mppi::Optimizer> getDummyOptimizer(
  TNode node, TCostMap costmap_ros,
  TParamHandler * params_handler)
{
  std::shared_ptr<mppi::Optimizer> optimizer = std::make_shared<mppi::Optimizer>();
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> weak_ptr_node{node};

  optimizer->initialize(weak_ptr_node, node->get_name(), costmap_ros, params_handler);

  return optimizer;
}

template<typename TNode, typename TCostMap, typename TFBuffer, typename TParamHandler>
mppi::PathHandler getDummyPathHandler(
  TNode node, TCostMap costmap_ros, TFBuffer tf_buffer,
  TParamHandler * params_handler)
{
  mppi::PathHandler path_handler;
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> weak_ptr_node{node};

  path_handler.initialize(weak_ptr_node, node->get_name(), costmap_ros, tf_buffer, params_handler);

  return path_handler;
}

template<typename TNode, typename TCostMap, typename TFBuffer>
std::shared_ptr<nav2_mppi_controller::MPPIController> getDummyController(
  TNode node, TFBuffer tf_buffer,
  TCostMap costmap_ros)
{
  auto controller = std::make_shared<nav2_mppi_controller::MPPIController>();
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> weak_ptr_node{node};

  controller->configure(weak_ptr_node, node->get_name(), tf_buffer, costmap_ros);
  controller->activate();
  return controller;
}

auto getDummyTwist()
{
  geometry_msgs::msg::Twist twist;
  return twist;
}

template<typename TNode>
geometry_msgs::msg::PoseStamped
getDummyPointStamped(TNode & node, std::string frame = std::string("odom"))
{
  geometry_msgs::msg::PoseStamped point;
  detail::setHeader(point, node, frame);

  return point;
}

template<typename TNode>
geometry_msgs::msg::PoseStamped getDummyPointStamped(TNode & node, TestPose pose)
{
  geometry_msgs::msg::PoseStamped point = getDummyPointStamped(node);
  point.pose.position.x = pose.x;
  point.pose.position.y = pose.y;

  return point;
}

template<typename TNode>
nav_msgs::msg::Path getDummyPath(TNode node, std::string frame = std::string("odom"))
{
  nav_msgs::msg::Path path;
  detail::setHeader(path, node, frame);
  return path;
}

template<typename TNode>
auto getDummyPath(size_t points_count, TNode node)
{
  auto path = getDummyPath(node);

  for (size_t i = 0; i < points_count; i++) {
    path.poses.push_back(getDummyPointStamped(node));
  }

  return path;
}

template<typename TNode>
nav_msgs::msg::Path getIncrementalDummyPath(TNode node, TestPathSettings s)
{
  auto path = getDummyPath(node);

  for (size_t i = 0; i < s.poses_count; i++) {
    double x = s.start_pose.x + static_cast<double>(i) * s.step_x;
    double y = s.start_pose.y + static_cast<double>(i) * s.step_y;
    path.poses.push_back(getDummyPointStamped(node, TestPose{x, y}));
  }

  return path;
}
