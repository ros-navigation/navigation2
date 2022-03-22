#pragma once

#include "nav2_costmap_2d/footprint_collision_checker.hpp"

#include "mppic/critic_function.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class ObstaclesCritic : public CriticFunction
{
public:
  void initialize() override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  virtual void score(
    const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
    const xt::xtensor<double, 3> & trajectories, const xt::xtensor<double, 2> & /*path*/,
    xt::xtensor<double, 1> & costs, nav2_core::GoalChecker * goal_checker) override;

protected:
  bool inCollision(unsigned char cost) const;
  double scoreCost(unsigned char cost);
  unsigned char maxCost();
  unsigned char costAtPose(const auto & point);

protected:
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker_;

  bool consider_footprint_{true};
  double inflation_cost_scaling_factor_{0};
  double inscribed_radius_{0};
  double inflation_radius_{0};
  unsigned int power_{0};
  double weight_{0};
};

} // namespace mppi::critics
