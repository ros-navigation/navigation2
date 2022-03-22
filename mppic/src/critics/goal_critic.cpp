#include "mppic/critics/goal_critic.hpp"

namespace mppi::critics
{

void GoalCritic::initialize()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, name_);

  getParam(power_, "goal_cost_power", 1);
  getParam(weight_, "goal_cost_weight", 20.0);
  RCLCPP_INFO(
    logger_, "GoalCritic instantiated with %d power and %f weight.", power_, weight_);
}

void GoalCritic::score(
  const geometry_msgs::msg::PoseStamped & /*robot_pose*/, const xt::xtensor<double,
  3> & trajectories,
  const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  const auto goal_points = xt::view(path, -1, xt::range(0, 2));

  auto trajectories_end = xt::view(trajectories, xt::all(), -1, xt::range(0, 2));

  auto dim = trajectories_end.dimension() - 1;

  auto && dists_trajectories_end_to_goal =
    xt::norm_l2(std::move(trajectories_end) - goal_points, {dim});

  costs += xt::pow(std::move(dists_trajectories_end_to_goal) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)
