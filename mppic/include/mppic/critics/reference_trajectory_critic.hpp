// Copyright 2022 FastSense, Samsung Research
#pragma once

#include "mppic/critic_function.hpp"
#include "mppic/utils.hpp"

namespace mppi::critics
{

class ReferenceTrajectoryCritic : public CriticFunction
{
public:
  void initialize() override;

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(
    const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
    const xt::xtensor<double, 3> & trajectories, const xt::xtensor<double, 2> & path,
    xt::xtensor<double, 1> & costs, nav2_core::GoalChecker * goal_checker) override;

  /**
    * @brief given reference_path [..., 2] and multiple trajectories [..., ..., 2],
    * evaluate mean distances from trajectories to reference_path [..., ..., ..., 2]
    *
    * @ref http://paulbourke.net/geometry/pointlineplane/
    */
  xt::xtensor<double, 1> meanDistancesFromTrajectoriesPointsToReferenceSegments(
    const xt::xtensor<double, 3> & trajectories, const xt::xtensor<double, 2> & reference_path);

protected:
  unsigned int power_{0};
  double weight_{0};
};

}  // namespace mppi::critics
