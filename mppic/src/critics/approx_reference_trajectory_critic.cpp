// Copyright 2022 FastSense, Samsung Research
#include "mppic/critics/approx_reference_trajectory_critic.hpp"

namespace mppi::critics
{

void ApproxReferenceTrajectoryCritic::initialize()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, name_);

  getParam(power_, "reference_cost_power", 1);
  getParam(weight_, "reference_cost_weight", 15.0);
  RCLCPP_INFO(
    logger_,
    "ApproxReferenceTrajectoryCritic instantiated with %d power and %f weight.",
    power_, weight_);
}

void ApproxReferenceTrajectoryCritic::score(
  const geometry_msgs::msg::PoseStamped & robot_pose, const xt::xtensor<double,
  3> & trajectories,
  const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs,
  nav2_core::GoalChecker * goal_checker)
{
  if (withinPositionGoalTolerance(goal_checker, robot_pose, path)) {
    return;
  }

  auto path_points = xt::view(path, xt::all(), xt::range(0, 2));
  auto trajectories_points_extended =
    xt::view(trajectories, xt::all(), xt::all(), xt::newaxis(), xt::range(0, 2));

  auto dists = xt::norm_l2(
    path_points - trajectories_points_extended, {trajectories_points_extended.dimension() - 1});
  auto && cost = xt::mean(xt::amin(std::move(dists), 1), 1);
  costs += xt::pow(std::move(cost) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::ApproxReferenceTrajectoryCritic,
  mppi::critics::CriticFunction)
