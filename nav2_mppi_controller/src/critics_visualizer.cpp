// Copyright (c) 2025 Open Navigation LLC
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

#include <cmath>
#include <vector>

#include "nav2_mppi_controller/tools/critics_visualizer.hpp"

namespace mppi
{

void CriticsVisualizer::on_configure(
  nav2::LifecycleNode::WeakPtr parent, const std::string & name,
  ParametersHandler * parameters_handler)
{
  parent_ = parent;
  parameters_handler_ = parameters_handler;
  auto node = parent_.lock();
  logger_ = node->get_logger();

  auto getParam = parameters_handler_->getParamGetter(name + ".Visualization");
  getParam(publish_critics_stats_, "publish_critics_stats", false, ParameterType::Static);
  getParam(
    visualize_per_critic_costs_, "publish_trajectories_with_individual_cost",
    false, ParameterType::Static);

  if (publish_critics_stats_) {
    critics_effect_pub_ = node->create_publisher<nav2_msgs::msg::CriticsStats>(
      "~/critics_stats");
    critics_effect_pub_->on_activate();
  }
}

void CriticsVisualizer::on_cleanup()
{
  critics_effect_pub_.reset();
}

void CriticsVisualizer::on_activate()
{
  if (critics_effect_pub_) {
    critics_effect_pub_->on_activate();
  }
}

void CriticsVisualizer::on_deactivate()
{
  if (critics_effect_pub_) {
    critics_effect_pub_->on_deactivate();
  }
}

bool CriticsVisualizer::shouldTrackCosts() const
{
  return publish_critics_stats_ || visualize_per_critic_costs_;
}

void CriticsVisualizer::prepareEvaluation(CriticData & data, size_t num_critics) const
{
  if (visualize_per_critic_costs_) {
    if (!data.individual_critics_cost) {
      data.individual_critics_cost = std::vector<std::pair<std::string, Eigen::ArrayXf>>();
    }
    data.individual_critics_cost->clear();
    data.individual_critics_cost->reserve(num_critics);
  }
}

void CriticsVisualizer::processEvaluation(
  const std::vector<std::string> & critic_names,
  const std::vector<Eigen::ArrayXf> & cost_diffs,
  CriticData & data) const
{
  if (!shouldTrackCosts() || cost_diffs.empty()) {
    return;
  }

  // Populate per-critic costs for trajectory visualization
  if (visualize_per_critic_costs_) {
    for (size_t i = 0; i < cost_diffs.size(); ++i) {
      data.individual_critics_cost->emplace_back(critic_names[i], cost_diffs[i]);
    }
  }

  // Compute and publish statistics
  if (!publish_critics_stats_) {
    return;
  }

  auto stats_msg = std::make_unique<nav2_msgs::msg::CriticsStats>();
  stats_msg->critics.reserve(cost_diffs.size());
  stats_msg->changed.reserve(cost_diffs.size());
  stats_msg->costs_std_dev.reserve(cost_diffs.size());
  stats_msg->influence_ratio.reserve(cost_diffs.size());

  for (size_t i = 0; i < cost_diffs.size(); ++i) {
    stats_msg->critics.push_back(critic_names[i]);
  }

  // Collect viable (non-collision) trajectory indices
  std::vector<int> viable_indices;
  const int n = data.costs.size();
  viable_indices.reserve(n);
  if (data.trajectory_collisions) {
    for (int j = 0; j < n; ++j) {
      if (!(*data.trajectory_collisions)(j)) {
        viable_indices.push_back(j);
      }
    }
  } else {
    for (int j = 0; j < n; ++j) {
      viable_indices.push_back(j);
    }
  }
  int viable_count = static_cast<int>(viable_indices.size());

  for (size_t i = 0; i < cost_diffs.size(); ++i) {
    if (viable_count > 1) {
      Eigen::ArrayXf viable_costs(viable_count);
      for (int k = 0; k < viable_count; ++k) {
        viable_costs(k) = cost_diffs[i](viable_indices[k]);
      }
      float mean = viable_costs.mean();
      float std_dev = std::sqrt((viable_costs - mean).square().mean());
      stats_msg->costs_std_dev.push_back(std_dev);
      stats_msg->changed.push_back(std_dev > 1e-6f);
    } else {
      stats_msg->costs_std_dev.push_back(0.0f);
      stats_msg->changed.push_back(false);
    }
  }

  // Compute influence ratios from std devs
  float total_std_dev = 0.0f;
  for (const auto & sd : stats_msg->costs_std_dev) {
    total_std_dev += sd;
  }
  for (const auto & sd : stats_msg->costs_std_dev) {
    stats_msg->influence_ratio.push_back(
      total_std_dev > 1e-6f ? sd / total_std_dev : 0.0f);
  }

  // Publish statistics
  if (critics_effect_pub_) {
    auto node = parent_.lock();
    stats_msg->stamp = node->get_clock()->now();
    critics_effect_pub_->publish(std::move(stats_msg));
  }
}

}  // namespace mppi
