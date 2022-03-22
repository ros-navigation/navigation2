// Copyright 2022 FastSense, Samsung Research
#include "mppic/critics/reference_trajectory_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{

void ReferenceTrajectoryCritic::initialize()
{
  auto node = parent_.lock();
  auto getParam = utils::getParamGetter(node, name_);
  getParam(power_, "reference_cost_power", 1);
  getParam(weight_, "reference_cost_weight", 15.0);
  RCLCPP_INFO(
    logger_, "ReferenceTrajectoryCritic instantiated with %d power and %f weight.", power_,
    weight_);
}

void ReferenceTrajectoryCritic::score(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const xt::xtensor<double, 3> & trajectories, const xt::xtensor<double, 2> & path,
  xt::xtensor<double, 1> & costs, nav2_core::GoalChecker * goal_checker)
{
  if (withinPositionGoalTolerance(goal_checker, robot_pose, path)) {
    return;
  }

  auto && distances = meanDistancesFromTrajectoriesPointsToReferenceSegments(trajectories, path);
  costs += xt::pow(std::move(distances) * weight_, power_);
}

xt::xtensor<double, 1>
ReferenceTrajectoryCritic::meanDistancesFromTrajectoriesPointsToReferenceSegments(
  const xt::xtensor<double, 3> & trajectories, const xt::xtensor<double, 2> & reference_path)
{
  using namespace xt::placeholders;  // NOLINT

  size_t trajectories_count = trajectories.shape()[0];
  size_t trajectories_points_count = trajectories.shape()[1];
  size_t reference_segments_count = reference_path.shape()[0] - 1;

  auto && distances = xt::xtensor<double, 1>::from_shape({trajectories_count});

  // see http://paulbourke.net/geometry/pointlineplane/
  const auto & P3 = trajectories;
  auto P1 = xt::view(reference_path, xt::all(), xt::range(_, -1));
  auto P2 = xt::view(reference_path, xt::all(), xt::range(1, _));
  xt::xtensor<double, 2> P2_P1_diff = P2 - P1;
  xt::xtensor<double, 1> P2_P1_norm_sq =
    xt::norm_sq(P2_P1_diff, {P2_P1_diff.dimension() - 1}, xt::evaluation_strategy::immediate);

  auto evaluate_u = [&P1, &P2, &P3, &P2_P1_diff,
      &P2_P1_norm_sq](size_t t, size_t p, size_t s) -> double {
      return ((P3(t, p, 0) - P1(s, 0)) * (P2_P1_diff(s, 0)) +
             (P3(t, p, 1) - P1(s, 1)) * (P2_P1_diff(s, 1))) /
             P2_P1_norm_sq(s);
    };

  static constexpr double eps = static_cast<double>(1e-3);  // meters
  auto segment_short = P2_P1_norm_sq < eps;
  auto evaluate_dist = [&P3](xt::xtensor_fixed<double, xt::xshape<2>> P, size_t t, size_t p) {
      double dx = P(0) - P3(t, p, 0);
      double dy = P(1) - P3(t, p, 1);
      return std::hypot(dx, dy);
    };

  for (size_t t = 0; t < trajectories_count; ++t) {
    double mean_dist = 0;
    for (size_t p = 0; p < trajectories_points_count; ++p) {
      double min_dist = std::numeric_limits<double>::max();
      for (size_t s = 0; s < reference_segments_count; ++s) {
        xt::xtensor_fixed<double, xt::xshape<2>> P;
        if (segment_short(s)) {
          P[0] = P1(s, 0);
          P[1] = P1(s, 1);
        } else if (double u = evaluate_u(t, p, s); u <= 0) {
          P[0] = P1(s, 0);
          P[1] = P1(s, 1);
        } else if (u >= 1) {
          P[0] = P2(s, 0);
          P[1] = P2(s, 1);
        } else {
          P[0] = P1(s, 0) + u * P2_P1_diff(s, 0);
          P[1] = P1(s, 1) + u * P2_P1_diff(s, 1);
        }
        min_dist = std::min(evaluate_dist(std::move(P), t, p), min_dist);
      }
      mean_dist += min_dist;
    }
    distances(t) = mean_dist / trajectories_points_count;
  }
  return distances;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ReferenceTrajectoryCritic, mppi::critics::CriticFunction)
