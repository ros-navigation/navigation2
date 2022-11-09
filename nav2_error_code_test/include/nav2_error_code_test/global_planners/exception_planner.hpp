//
// Created by josh on 11/9/22.
//

#ifndef NAV2_WS_EXCEPTION_PLANNER_HPP
#define NAV2_WS_EXCEPTION_PLANNER_HPP

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

namespace nav2_error_code_test
{

class ExceptionPlanner : public nav2_core::GlobalPlanner
{
public:
  ExceptionPlanner() = default;
  ~ExceptionPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {};

  void cleanup() {};

  void activate() {};

  void deactivate() {};

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal);


};
}

#endif //NAV2_WS_EXCEPTION_PLANNER_HPP
