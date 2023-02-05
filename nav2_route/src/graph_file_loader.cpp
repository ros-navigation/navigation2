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
#include <filesystem>

#include "nav2_route/graph_file_loader.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

// TODO(jw): move into plugin
#include <fstream>

namespace nav2_route
{

GraphFileLoader::GraphFileLoader(
  nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame)
  : plugin_loader_("nav2_route", "nav2_route::GraphParser"),
    default_plugin_id_({"GeoJsonGraphParser"})
{
  logger_ = node->get_logger();
  tf_ = tf;
  route_frame_ = frame;

  // Default Graph Parser
  const std::string default_plugin_type = "nav2_route::GeoJsonGraphParser";

  nav2_util::declare_parameter_if_not_declared(
      node, "graph_parsers", rclcpp::ParameterValue(default_plugin_id_));
  auto graph_parser_ids = node->get_parameter("graph_parsers").as_string_array();

  if (graph_parser_ids == default_plugin_id_) {
    nav2_util::declare_parameter_if_not_declared(
        node, default_plugin_id_[0] + ".plugin", rclcpp::ParameterValue(default_plugin_type));
  }

  // Create plugins
  for (size_t i = 0; i != graph_parser_ids.size(); ++i)
  {
    try {
      std::string type = nav2_util::get_plugin_type_param(node, graph_parser_ids[i]);
      GraphParser::Ptr graph_parser = plugin_loader_.createSharedInstance((type));
      RCLCPP_INFO(
          logger_, "Created route operation %s of type %s",
          graph_parser_ids[i].c_str(), type.c_str());
      graph_parsers_.insert({graph_parser_ids[i], std::move(graph_parser)});
    }
    catch ( pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
          logger_,
          "Failed to create graph parser. Exception: %s", ex.what());
      throw ex;
    }
  }
}

bool GraphFileLoader::loadGraphFromFile(
    Graph &graph,
    GraphToIDMap &,
    const std::string& filepath,
    std::string parser_id)
{
   if (!fileExists(filepath)) {
     RCLCPP_ERROR(logger_, "Graph file %s does not exist!", filepath.c_str());
     return false;
   }

   if (parser_id.empty()) {
     RCLCPP_WARN(logger_, "Parser id was unset, setting to %s", default_plugin_id_[0].c_str());
     parser_id = default_plugin_id_[0];
   }

   bool result = false;
   if ( graph_parsers_.find(parser_id) != graph_parsers_.end()) {
     try {
       result = graph_parsers_[parser_id]->loadGraphFromFile(graph, filepath);
     } catch (std::exception &ex) {
       throw ex;
     }
   }

   return result;

  // Validate file is legit using a plugin API TODO(sm)
  // Load file using a plugin API

  // Convert all coordinates to `frame` (in a new method) for standardization
  // Including conversion of GPS coordinates, so we can populate it in some
  // cartesian frame necessary for traversal cost estimation and densifying
  // (and so we don't need to propogate it through our structures)

}

bool GraphFileLoader::fileExists(const std::string & filepath)
{
  return std::filesystem::exists(filepath);
}

}  // namespace nav2_route
