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

#ifndef NAV2_ROUTE__EDGE_SCORER_HPP_
#define NAV2_ROUTE__EDGE_SCORER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/interfaces/edge_cost_function.hpp"

namespace nav2_route
{

/**
 * @class nav2_route::EdgeScorer
 * @brief An class to encapsulate edge scoring logic for plugins and different user
 * specified algorithms to influence graph search. It has access to the edge, which
 * in turn has access to the parent and child node of the connection. It also contains
 * action and arbitrary user-defined metadata to enable edge scoring logic based on
 * arbitrary properties of the graph you select (e.g. some regions have a multipler,
 * some actions are discouraged with higher costs like having to go through a door,
 * edges with reduced speed limits are proportionally less preferred for optimality
 * relative to the distance the edge represents to optimize time to goal)
 */
class EdgeScorer
{
public:
  /**
   * @brief Constructor
   */
  EdgeScorer(nav2_util::LifecycleNode::SharedPtr node)
  : plugin_loader_("nav2_route", "nav2_route::EdgeCostFunction")
  {
    // load plugins with a default of the DistanceScorer
    const std::vector<std::string> default_plugin_id({"DistanceScorer"});
    const std::string default_plugin_type = "nav2_route::DistanceScorer";

    nav2_util::declare_parameter_if_not_declared(
      node, "edge_cost_functions", rclcpp::ParameterValue(default_plugin_id));
    auto edge_cost_function_ids = node->get_parameter("edge_cost_functions").as_string_array();

    if (edge_cost_function_ids == default_plugin_id) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_plugin_id[0] + ".plugin", rclcpp::ParameterValue(default_plugin_type));
    }

    for (size_t i = 0; i != edge_cost_function_ids.size(); i++) {
      try {
        std::string type = nav2_util::get_plugin_type_param(
          node, edge_cost_function_ids[i]);
        EdgeCostFunction::Ptr scorer = plugin_loader_.createUniqueInstance(type);
        RCLCPP_INFO(
          node->get_logger(), "Created edge cost function plugin %s of type %s",
          edge_cost_function_ids[i].c_str(), type.c_str());
        scorer->configure(node, edge_cost_function_ids[i]);  // TODO TF, costmap?
        plugins_.push_back(std::move(scorer));
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          node->get_logger(),
          "Failed to create edge cost function. Exception: %s", ex.what());
        exit(-1);
      }
    }
  }

  /**
   * @brief Destructor
   */
  ~EdgeScorer() = default;

  float score(const EdgePtr edge)
  {
    float score = 0.0;
    for (auto & plugin : plugins_) {
      score += plugin->score(edge);
    }

    return score;
  }

  /**
   * @brief Provide the number of plugisn in the scorer loaded
   * @return Number of scoring plugins
   */
  int numPlugins() const
  {
    return plugins_.size();
  }

protected:
  pluginlib::ClassLoader<EdgeCostFunction> plugin_loader_;
  std::vector<EdgeCostFunction::Ptr> plugins_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__EDGE_SCORER_HPP_
