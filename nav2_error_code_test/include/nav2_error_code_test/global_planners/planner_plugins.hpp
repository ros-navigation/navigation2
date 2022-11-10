//
// Created by josh on 11/10/22.
//

#ifndef NAV2_WS_PLANNER_PLUGINS_HPP
#define NAV2_WS_PLANNER_PLUGINS_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "unknown_error_planner.hpp"

namespace nav2_error_code_test
{

class StartOccupiedErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override
  {
    throw nav2_core::StartOccupied("Start Occupied");
  }
};

class GoalOccupiedErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override
  {
    throw nav2_core::GoalOccupied("Goal occupied");
  }
};

class StartOutsideMapErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override
  {
    throw nav2_core::StartOutsideMapBounds("Start OutsideMapBounds");
  }
};

class GoalOutsideMapErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override
  {
    throw nav2_core::GoalOutsideMapBounds("Goal outside map bounds");
  }
};

class NoValidPathErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override
  {
    throw nav2_core::NoValidPathCouldBeFound("No valid path could be found");
  }
};


class TimedOutErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    throw nav2_core::PlannerTimedOut("Planner Timed Out");
  }
};

class TFErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override
  {
    throw nav2_core::PlannerTFError("TF Error");
  }
};

class NoViapointsGivenErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override
  {
    throw nav2_core::NoViapointsGiven("No Via points given");
  }
};
}


#endif //NAV2_WS_PLANNER_PLUGINS_HPP
