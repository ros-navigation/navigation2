// Copyright (c) 2024 John Chrosniak
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

#include "nav2_route/graph_saver.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace nav2_route
{

GraphSaver::GraphSaver(
  nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame)
: plugin_loader_("nav2_route", "nav2_route::GraphFileSaver"),
  default_plugin_id_("GeoJsonGraphFileSaver")
{
  logger_ = node->get_logger();
  tf_ = tf;
  route_frame_ = frame;

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_filepath", rclcpp::ParameterValue(std::string("")));
  graph_filepath_ = node->get_parameter("graph_filepath").as_string();

  // Default Graph Parser
  const std::string default_plugin_type = "nav2_route::GeoJsonGraphFileSaver";
  nav2_util::declare_parameter_if_not_declared(
    node, "graph_file_saver", rclcpp::ParameterValue(default_plugin_id_));
  auto graph_file_saver_id = node->get_parameter("graph_file_saver").as_string();
  if (graph_file_saver_id == default_plugin_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_plugin_id_ + ".plugin", rclcpp::ParameterValue(default_plugin_type));
  }

  // Create graph file saver plugin
  try {
    plugin_type_ = nav2_util::get_plugin_type_param(node, graph_file_saver_id);
    graph_file_saver_ = plugin_loader_.createSharedInstance((plugin_type_));
    RCLCPP_INFO(
      logger_, "Created GraphFileSaver %s of type %s",
      graph_file_saver_id.c_str(), plugin_type_.c_str());
    graph_file_saver_->configure(node);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_,
      "Failed to create GraphFileSaver. Exception: %s", ex.what());
    throw ex;
  }
}

bool GraphSaver::saveGraphToFile(
  Graph & graph,
  std::string filepath)
{
  if (filepath.empty() && !graph_filepath_.empty()) {
    RCLCPP_DEBUG(
      logger_, "The graph filepath was not provided. "
      "Setting to %s", graph_filepath_.c_str());
    filepath = graph_filepath_;
  } else if (filepath.empty() && graph_filepath_.empty()) {
    // No graph to try to save
    RCLCPP_WARN(
      logger_,
      "The graph filepath was not provided and no default was specified. "
      "Failed to save the route graph.");
    return false;
  }

  RCLCPP_INFO(
    logger_,
    "Saving graph file from %s, by parser %s", filepath.c_str(), plugin_type_.c_str());

  if (!graph_file_saver_->saveGraphToFile(graph, filepath)) {
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

bool GraphSaver::transformGraph(Graph & graph)
{
  std::unordered_map<std::string, tf2::Transform> cached_transforms;
  for (auto & node : graph) {
    std::string node_frame = node.coords.frame_id;
    if (node_frame.empty() || node_frame == route_frame_) {
      // If there is no frame provided or the frame of the node is the same as the route graph
      // then no transform is required
      continue;
    }
    // Find the transform to convert node from node frame to route frame
    if (cached_transforms.find(node_frame) == cached_transforms.end()) {
      tf2::Transform tf_transform;
      bool got_transform = nav2_util::getTransform(
        node_frame, route_frame_, tf2::durationFromSec(0.1), tf_, tf_transform);

      if (!got_transform) {
        RCLCPP_WARN(
          logger_,
          "Could not get transform from node frame %s to route frame %s",
          node_frame.c_str(), route_frame_.c_str());
        return false;
      }

      cached_transforms.insert({node_frame, tf_transform});
    }
    // Apply the transform
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
