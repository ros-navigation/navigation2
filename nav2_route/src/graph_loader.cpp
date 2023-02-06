// Copyright (c) 2023, Samsung Research America
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
  default_plugin_id_({"GeoJsonGraphFileLoader"})
{
  logger_ = node->get_logger();
  tf_ = tf;
  route_frame_ = frame;

  // Default Graph Parser
  const std::string default_plugin_type = "nav2_route::GeoJsonGraphFileLoader";

  nav2_util::declare_parameter_if_not_declared(
    node, "graph_file_loaders", rclcpp::ParameterValue(default_plugin_id_));
  auto graph_file_loader_ids = node->get_parameter("graph_file_loaders").as_string_array();

  if (graph_file_loader_ids == default_plugin_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_plugin_id_[0] + ".plugin", rclcpp::ParameterValue(default_plugin_type));
  }

  // Create plugins
  for (size_t i = 0; i != graph_file_loader_ids.size(); ++i) {
    try {
      std::string type = nav2_util::get_plugin_type_param(node, graph_file_loader_ids[i]);
      GraphFileLoader::Ptr graph_parser = plugin_loader_.createSharedInstance((type));
      RCLCPP_INFO(
        logger_, "Created GraphFileLoader %s of type %s",
        graph_file_loader_ids[i].c_str(), type.c_str());
      graph_parsers_.insert({graph_file_loader_ids[i], std::move(graph_parser)});
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        logger_,
        "Failed to create graph parser. Exception: %s", ex.what());
      throw ex;
    }
  }
}

bool GraphLoader::loadGraphFromFile(
  Graph & graph,
  GraphToIDMap & graph_to_id_map,
  const std::string & filepath,
  std::string parser_id)
{
  if (parser_id.empty()) {
    RCLCPP_WARN(logger_, "Parser id was unset, setting to %s", default_plugin_id_[0].c_str());
    parser_id = default_plugin_id_[0];
  }

  bool result = false;
  if (graph_parsers_.find(parser_id) != graph_parsers_.end()) {
    if ( !graph_parsers_[parser_id]->fileExists(filepath))
    {
      RCLCPP_ERROR(logger_, "The filepath %s does not exist", filepath.c_str());
      return false;
    }
    result = graph_parsers_[parser_id]->loadGraphFromFile(graph, graph_to_id_map, filepath);
  }

  return result;

  // Validate file is legit using a plugin API TODO(sm)
  // Load file using a plugin API

  // Convert all coordinates to `frame` (in a new method) for standardization
  // Including conversion of GPS coordinates, so we can populate it in some
  // cartesian frame necessary for traversal cost estimation and densifying
  // (and so we don't need to propogate it through our structures)
}
}  // namespace nav2_route
