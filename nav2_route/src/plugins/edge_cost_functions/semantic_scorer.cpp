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
#include <string>

#include "nav2_route/plugins/edge_cost_functions/semantic_scorer.hpp"

namespace nav2_route
{

void SemanticScorer::configure(
  const nav2_util::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer>/* tf_buffer */,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring semantic scorer.");
  name_ = name;

  // Find the semantic data
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".semantic_classes", rclcpp::ParameterValue(std::vector<std::string>{}));
  std::vector<std::string> classes =
    node->get_parameter(getName() + ".semantic_classes").as_string_array();
  for (auto & cl : classes) {
    nav2_util::declare_parameter_if_not_declared(
      node, getName() + "." + cl, rclcpp::ParameterType::PARAMETER_DOUBLE);
    const double cost = node->get_parameter(getName() + "." + cl).as_double();
    semantic_info_[cl] = static_cast<float>(cost);
  }

  // Find the key to look for semantic data for within the metadata. If set to empty string,
  // will search instead for any key in the metadata.
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".semantic_key", rclcpp::ParameterValue(std::string("class")));
  key_ = node->get_parameter(getName() + ".semantic_key").as_string();

  // Find the proportional weight to apply, if multiple cost functions
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".weight", rclcpp::ParameterValue(1.0));
  weight_ = static_cast<float>(node->get_parameter(getName() + ".weight").as_double());
}

void SemanticScorer::metadataKeyScorer(Metadata & mdata, float & score)
{
  for (auto it = mdata.data.cbegin(); it != mdata.data.cend(); ++it) {
    if (auto sem = semantic_info_.find(it->first); sem != semantic_info_.end()) {
      score += sem->second;
    }
  }
}

void SemanticScorer::metadataValueScorer(Metadata & mdata, float & score)
{
  std::string cl;
  cl = mdata.getValue<std::string>(key_, cl);
  if (auto sem = semantic_info_.find(cl); sem != semantic_info_.end()) {
    score += sem->second;
  }
}

bool SemanticScorer::score(
  const EdgePtr edge,
  const RouteRequest & /* route_request */,
  const EdgeType & /* edge_type */, float & cost)
{
  float score = 0.0;
  Metadata & node_mdata = edge->end->metadata;
  Metadata & edge_mdata = edge->metadata;

  // If a particular key is known to have semantic info, use it, else search
  // each metadata key field to see if it matches
  if (key_.empty()) {
    metadataKeyScorer(node_mdata, score);
    metadataKeyScorer(edge_mdata, score);
  } else {
    metadataValueScorer(node_mdata, score);
    metadataValueScorer(edge_mdata, score);
  }

  cost = weight_ * score;
  return true;
}

std::string SemanticScorer::getName()
{
  return name_;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::SemanticScorer, nav2_route::EdgeCostFunction)
