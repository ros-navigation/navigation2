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

#include <cmath>
#include <algorithm>
#include <vector>

#include "nav2_mppi_controller/critic_manager.hpp"

namespace mppi
{

void CriticManager::on_configure(
  nav2::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);

  // Read visualization parameters from Visualization namespace
  auto getVisualizerParam = parameters_handler_->getParamGetter(name_ + ".Visualization");
  getVisualizerParam(publish_critics_stats_, "publish_critics_stats", false, ParameterType::Static);
  getVisualizerParam(visualize_per_critic_costs_, "publish_trajectories_with_individual_cost",
      false, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
  }

  auto node = parent_.lock();
  if (publish_critics_stats_) {
    critics_effect_pub_ = node->create_publisher<nav2_msgs::msg::CriticsStats>(
      "~/critics_stats");
    critics_effect_pub_->on_activate();
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  std::unique_ptr<nav2_msgs::msg::CriticsStats> stats_msg;
  std::vector<Eigen::ArrayXf> critic_cost_diffs;  // for deferred stats computation
  if (publish_critics_stats_) {
    stats_msg = std::make_unique<nav2_msgs::msg::CriticsStats>();
    stats_msg->critics.reserve(critics_.size());
    stats_msg->changed.reserve(critics_.size());
    stats_msg->costs_std_dev.reserve(critics_.size());
    stats_msg->influence_ratio.reserve(critics_.size());
    critic_cost_diffs.reserve(critics_.size());
  }

  // Initialize per-critic costs tracking only if requested
  if (visualize_per_critic_costs_) {
    if (!data.individual_critics_cost) {
      data.individual_critics_cost = std::vector<std::pair<std::string, Eigen::ArrayXf>>();
    }
    data.individual_critics_cost->clear();
    data.individual_critics_cost->reserve(critics_.size());
  }

  for (size_t i = 0; i < critics_.size(); ++i) {
    if (data.fail_flag) {
      break;
    }

    // Store costs before critic evaluation
    Eigen::ArrayXf costs_before;
    if (visualize_per_critic_costs_ || publish_critics_stats_) {
      costs_before = data.costs;
    }

    critics_[i]->score(data);

    // Calculate cost contribution from this critic
    if (visualize_per_critic_costs_ || publish_critics_stats_) {
      Eigen::ArrayXf cost_diff = data.costs - costs_before;

      if (visualize_per_critic_costs_) {
        data.individual_critics_cost->emplace_back(critic_names_[i], cost_diff);
      }

      if (publish_critics_stats_) {
        stats_msg->critics.push_back(critic_names_[i]);
        critic_cost_diffs.push_back(cost_diff);
      }
    }
  }

  // Compute stats excluding collision trajectories using the mask from CostCritic
  if (publish_critics_stats_ && !critic_cost_diffs.empty()) {
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

    for (size_t i = 0; i < critic_cost_diffs.size(); ++i) {
      if (viable_count > 1) {
        Eigen::ArrayXf viable_costs(viable_count);
        for (int k = 0; k < viable_count; ++k) {
          viable_costs(k) = critic_cost_diffs[i](viable_indices[k]);
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
  }

  // Publish statistics if enabled
  if (critics_effect_pub_ && stats_msg) {
    auto node = parent_.lock();
    stats_msg->stamp = node->get_clock()->now();
    critics_effect_pub_->publish(std::move(stats_msg));
  }
}

}  // namespace mppi
