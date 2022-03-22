#ifndef MPPIC__CRITIC_FUNCTION_HPP_
#define MPPIC__CRITIC_FUNCTION_HPP_

#include <string>
#include <xtensor/xtensor.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xview.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mppi::critics
{
class CriticFunction
{
public:
  CriticFunction() = default;
  virtual ~CriticFunction() = default;

  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    parent_ = parent;
    logger_ = parent_.lock()->get_logger();
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    initialize();
  }

  virtual void initialize() = 0;

  virtual void score(
    const geometry_msgs::msg::PoseStamped & robot_pose, const xt::xtensor<double, 3> & trajectories,
    const xt::xtensor<double, 2> & path, xt::xtensor<double, 1> & costs,
    nav2_core::GoalChecker * goal_checker) = 0;

protected:
  bool withinPositionGoalTolerance(
    nav2_core::GoalChecker * goal_checker,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const xt::xtensor<double, 2> & path)
  {
    if (goal_checker) {
      geometry_msgs::msg::Pose pose_tol;
      geometry_msgs::msg::Twist vel_tol;
      goal_checker->getTolerances(pose_tol, vel_tol);

      const double & goal_tol = pose_tol.position.x;

      xt::xtensor<double, 1> tensor_pose = {
        static_cast<double>(robot_pose.pose.position.x),
        static_cast<double>(robot_pose.pose.position.y)};
      auto path_points = xt::view(path, -1, xt::range(0, 2));

      double dist_to_goal = xt::norm_l2(tensor_pose - path_points, {0})();

      if (dist_to_goal < goal_tol) {
        return true;
      }
    }

    return false;
  }

  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

} // namespace mppi::critics

#endif  // MPPIC__CRITIC_FUNCTION_HPP_
