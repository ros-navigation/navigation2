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
  getParam(publish_critics_stats_, "publish_critics_stats", false, ParameterType::Static);
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
  CriticData & data)
{
  const bool need_per_critic = store_per_critic_costs_ || publish_critics_stats_;

  if (need_per_critic) {
    // Zero-store-sum approach: zero costs before each critic, store its
    // individual contribution, then sum all at the end.
    per_critic_costs_.clear();
    per_critic_costs_.reserve(critics_.size());
    Eigen::ArrayXf total = Eigen::ArrayXf::Zero(data.costs.size());

    for (size_t i = 0; i < critics_.size(); ++i) {
      if (data.fail_flag) {
        break;
      }
      data.costs.setZero();
      critics_[i]->score(data);
      per_critic_costs_.emplace_back(critic_names_[i], data.costs);
      total += data.costs;
    }
    data.costs = total;
  } else {
    // Original fast path: no per-critic tracking overhead
    for (size_t i = 0; i < critics_.size(); ++i) {
      if (data.fail_flag) {
        break;
      }
      critics_[i]->score(data);
    }
  }

  if (publish_critics_stats_ && critics_effect_pub_) {
    auto stats_msg = std::make_unique<nav2_msgs::msg::CriticsStats>();
    const size_t n = per_critic_costs_.size();
    stats_msg->critics.reserve(n);
    stats_msg->changed.reserve(n);
    stats_msg->costs_sum.reserve(n);

    for (size_t i = 0; i < n; ++i) {
      const auto & [name, costs] = per_critic_costs_[i];
      stats_msg->critics.push_back(name);

      float sum = costs.sum();
      stats_msg->costs_sum.push_back(sum);
      stats_msg->changed.push_back(sum != 0.0f);
    }

    auto node = parent_.lock();
    stats_msg->stamp = node->get_clock()->now();
    critics_effect_pub_->publish(std::move(stats_msg));
  }
}

}  // namespace mppi
