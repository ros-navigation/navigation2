#include "mppic/critics/goal_angle_critic.hpp"

namespace mppi::critics
{

void GoalAngleCritic::initialize()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, name_);

  getParam(power_, "goal_angle_cost_power", 1);
  getParam(weight_, "goal_angle_cost_weight", 15.0);
  getParam(threshold_to_consider_goal_angle_, "threshold_to_consider_goal_angle", 0.30);
  RCLCPP_INFO(
    logger_,
    "GoalAngleCritic instantiated with %d power, %f weight, and %f angular threshold.",
    power_, weight_, threshold_to_consider_goal_angle_);
}

void GoalAngleCritic::score(
  const geometry_msgs::msg::PoseStamped & robot_pose, const xt::xtensor<double, 3> & trajectories,
  const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  xt::xtensor<double, 1> tensor_pose = {
    static_cast<double>(robot_pose.pose.position.x),
    static_cast<double>(robot_pose.pose.position.y)};

  auto path_points = xt::view(path, -1, xt::range(0, 2));

  double points_to_goal_dists = xt::norm_l2(tensor_pose - path_points, {0})();

  if (points_to_goal_dists < threshold_to_consider_goal_angle_) {
    auto yaws = xt::view(trajectories, xt::all(), xt::all(), 2);
    auto goal_yaw = xt::view(path, -1, 2);

    costs += xt::pow(xt::mean(xt::abs(yaws - goal_yaw), {1}) * weight_, power_);
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalAngleCritic, mppi::critics::CriticFunction)
