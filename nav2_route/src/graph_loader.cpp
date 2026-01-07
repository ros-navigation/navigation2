// Copyright (c) 2025, Open Navigation LLC
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

#include <memory>

#include "nav2_route/graph_loader.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace nav2_route
{

GraphLoader::GraphLoader(
  nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame)
: plugin_loader_("nav2_route", "nav2_route::GraphFileLoader"),
  default_plugin_id_("GeoJsonGraphFileLoader")
{
  logger_ = node->get_logger();
  tf_ = tf;
  route_frame_ = frame;

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(std::string("")));
  graph_filepath_ = node->get_parameter("graph_filepath").as_string();

  // Default Graph Parser
  nav2_util::declare_parameter_if_not_declared(
    node, "graph_file_loader", rclcpp::ParameterValue(default_plugin_id_));
  auto graph_file_loader_id = node->get_parameter("graph_file_loader").as_string();
  if (graph_file_loader_id == default_plugin_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_plugin_id_ + ".plugin",
      rclcpp::ParameterValue("nav2_route::GeoJsonGraphFileLoader"));
  }

  // Create graph file loader plugin
  try {
    plugin_type_ = nav2_util::get_plugin_type_param(node, graph_file_loader_id);
    graph_file_loader_ = plugin_loader_.createSharedInstance((plugin_type_));
    RCLCPP_INFO(
      logger_, "Created GraphFileLoader %s of type %s",
      graph_file_loader_id.c_str(), plugin_type_.c_str());
    graph_file_loader_->configure(node);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_,
      "Failed to create GraphFileLoader. Exception: %s", ex.what());
    throw ex;
  }
}

bool GraphLoader::loadGraphFromFile(
  Graph & graph,
  GraphToIDMap & graph_to_id_map,
  const std::string & filepath)
{
  if (filepath.empty()) {
    RCLCPP_ERROR(
      logger_, "The graph filepath was not provided.");
    return false;
  }

  RCLCPP_INFO(
    logger_,
    "Loading graph file from %s, by parser %s", filepath.c_str(), plugin_type_.c_str());

  if (!graph_file_loader_->loadGraphFromFile(graph, graph_to_id_map, filepath)) {
    return false;
  }

  if (!transformGraph(graph)) {
    RCLCPP_WARN(
      logger_,
      "Failed to transform nodes graph file (%s) to %s!", filepath.c_str(), route_frame_.c_str());
    return false;
  }

  return true;
}

bool GraphLoader::loadGraphFromParameter(
  Graph & graph,
  GraphToIDMap & graph_to_id_map)
{
  if (graph_filepath_.empty()) {
    RCLCPP_INFO(logger_, "No graph file provided to load yet.");
    return true;
  }

  RCLCPP_INFO(
    logger_,
    "Loading graph file from %s, by parser %s", graph_filepath_.c_str(), plugin_type_.c_str());

  if (!graph_file_loader_->loadGraphFromFile(graph, graph_to_id_map, graph_filepath_)) {
    return false;
  }

  if (!transformGraph(graph)) {
    RCLCPP_WARN(
      logger_,
      "Failed to transform nodes graph file (%s) to %s!",
      graph_filepath_.c_str(), route_frame_.c_str());
    return false;
  }

  return true;
}

bool GraphLoader::transformGraph(Graph & graph)
{
  std::unordered_map<std::string, tf2::Transform> cached_transforms;
  for (auto & node : graph) {
    std::string node_frame = node.coords.frame_id;
    if (node_frame.empty() || node_frame == route_frame_) {
      continue;
    }

    if (cached_transforms.find(node_frame) == cached_transforms.end()) {
      tf2::Transform tf_transform;
      bool got_transform = nav2_util::getTransform(
        node_frame, route_frame_, tf2::durationFromSec(0.1), tf_, tf_transform);

      if (!got_transform) {
        return false;
      }

      cached_transforms.insert({node_frame, tf_transform});
    }

    tf2::Vector3 graph_coord(
      node.coords.x,
      node.coords.y,
      0.0);

    tf2::Vector3 new_coord = cached_transforms[node_frame] * graph_coord;

    node.coords.x = static_cast<float>(new_coord.x());
    node.coords.y = static_cast<float>(new_coord.y());
    node.coords.frame_id = route_frame_;
  }

  return true;
}

}  // namespace nav2_route
