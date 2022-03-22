// Copyright 2022 FastSense, Samsung Research
#include "mppic/critic_manager.hpp"

namespace mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, name_);
  getParam(critic_names_, "critics", std::vector<std::string>{});
}

void CriticManager::loadCritics()
{
  loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
    "mppic", "mppi::critics::CriticFunction");

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance =
      std::unique_ptr<critics::CriticFunction>(loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(parent_, name_ + "." + name, costmap_ros_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

xt::xtensor<double, 1> CriticManager::evalTrajectoriesScores(
  const xt::xtensor<double, 3> & trajectories,
  const nav_msgs::msg::Path & global_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  nav2_core::GoalChecker * goal_checker) const
{
  // Create evalated costs tensor
  size_t trajectories_count = trajectories.shape()[0];
  xt::xtensor<double, 1> costs = xt::zeros<double>({trajectories_count});

  if (global_plan.poses.empty()) {
    return costs;
  }

  // Transform path into tensor for evaluation
  xt::xtensor<double, 2> path = utils::toTensor(global_plan);

  // Evaluate each trajectory by the critics
  for (size_t q = 0; q < critics_.size(); q++) {
    critics_[q]->score(robot_pose, trajectories, path, costs, goal_checker);
  }

  return costs;
}

}  // namespace mppi
