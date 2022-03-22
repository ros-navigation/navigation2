// Copyright 2022 FastSense, Samsung Research
#include "mppic/critics/obstacles_critic.hpp"
#include <xtensor/xaxis_slice_iterator.hpp>

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, name_);
  getParam(consider_footprint_, "consider_footprint", true);
  getParam(power_, "obstacle_cost_power", 1);
  getParam(weight_, "obstacle_cost_weight", 1.0);

  inscribed_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  collision_checker_.setCostmap(costmap_);
  RCLCPP_INFO(
    logger_,
    "ObstaclesCritic instantiated with %d power and %f weight. "
    "Critic will collision check based on %s cost.",
    power_, weight_, consider_footprint_ ? "footprint" : "circular");
}

void ObstaclesCritic::score(
  const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
  const xt::xtensor<double, 3> & trajectories, const xt::xtensor<double, 2> & /*path*/,
  xt::xtensor<double, 1> & costs, nav2_core::GoalChecker * /*goal_checker*/)
{
  constexpr double COLLISION_COST = std::numeric_limits<double>::max() / 4;

  for (size_t i = 0; i < trajectories.shape()[0]; ++i) {
    bool trajectory_collide = false;

    unsigned char trajectory_cost = nav2_costmap_2d::FREE_SPACE;
    for (size_t j = 0; j < trajectories.shape()[1]; ++j) {
      auto point = xt::view(trajectories, i, j, xt::all());
      unsigned char pose_cost = costAtPose(point);
      trajectory_cost = std::max(trajectory_cost, pose_cost);

      if (inCollision(pose_cost)) {
        trajectory_collide = true;
        break;
      }
    }

    costs[i] += trajectory_collide ? COLLISION_COST : scoreCost(trajectory_cost);
  }
}

unsigned char ObstaclesCritic::costAtPose(const auto & point)
{
  unsigned char cost;
  if (consider_footprint_) {
    cost = static_cast<unsigned char>(collision_checker_.footprintCostAtPose(
        point(0), point(1), point(2), costmap_ros_->getRobotFootprint()));
  } else {
    cost = static_cast<unsigned char>(collision_checker_.pointCost(point(0), point(1)));
  }
  return cost;
}

bool ObstaclesCritic::inCollision(unsigned char cost) const
{
  bool is_tracking_unknown = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

  switch (cost) {
    case (nav2_costmap_2d::LETHAL_OBSTACLE):
      return true;
    case (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint_ ? false : true;
    case (nav2_costmap_2d::NO_INFORMATION):
      return is_tracking_unknown ? false : true;
    default:
      return false;
  }
}

unsigned char ObstaclesCritic::maxCost()
{
  return consider_footprint_ ? nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE :
         nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
}

double ObstaclesCritic::scoreCost(unsigned char cost_arg)
{
  double max_cost = static_cast<double>(maxCost());

  double cost = static_cast<double>(cost_arg) / max_cost;

  return pow(cost * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ObstaclesCritic, mppi::critics::CriticFunction)
