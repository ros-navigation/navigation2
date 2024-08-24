// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>
#include <memory>
#include <string>
#include "nav2_theta_star_planner/theta_star_planner.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"

namespace nav2_theta_star_planner
{
void ThetaStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<theta_star::ThetaStar>();
  parent_node_ = parent;
  auto node = parent_node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".how_many_corners", rclcpp::ParameterValue(8));

  node->get_parameter(name_ + ".how_many_corners", planner_->how_many_corners_);

  if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
    planner_->how_many_corners_ = 8;
    RCLCPP_WARN(logger_, "Your value for - .how_many_corners  was overridden, and is now set to 8");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_unknown", planner_->allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_euc_cost", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".w_euc_cost", planner_->w_euc_cost_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(2.0));
  node->get_parameter(name_ + ".w_traversal_cost", planner_->w_traversal_cost_);

  planner_->w_heuristic_cost_ = planner_->w_euc_cost_ < 1.0 ? planner_->w_euc_cost_ : 1.0;

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".terminal_checking_interval", rclcpp::ParameterValue(5000));
  node->get_parameter(name_ + ".terminal_checking_interval", planner_->terminal_checking_interval_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);
}

void ThetaStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type nav2_theta_star_planner", name_.c_str());
  planner_.reset();
}

void ThetaStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type nav2_theta_star_planner", name_.c_str());
  // Add callback for dynamic parameters
  auto node = parent_node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ThetaStarPlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void ThetaStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type nav2_theta_star_planner", name_.c_str());
  auto node = parent_node_.lock();
  if (node && dyn_params_handler_) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

nav_msgs::msg::Path ThetaStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path global_path;
  auto start_time = std::chrono::steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(planner_->costmap_->getMutex()));

  // Corner case of start and goal beeing on the same cell
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!planner_->costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, mx_start, my_start))
  {
    throw nav2_core::StartOutsideMapBounds(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  if (!planner_->costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal))
  {
    throw nav2_core::GoalOutsideMapBounds(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }

  if (planner_->costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::GoalOccupied(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was in lethal cost");
  }

  if (mx_start == mx_goal && my_start == my_goal) {
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    global_path.poses.push_back(pose);
    return global_path;
  }

  planner_->clearStart();
  planner_->setStartAndGoal(start, goal);
  RCLCPP_DEBUG(
    logger_, "Got the src and dst... (%i, %i) && (%i, %i)",
    planner_->src_.x, planner_->src_.y, planner_->dst_.x, planner_->dst_.y);
  getPlan(global_path, cancel_checker);
  // check if a plan is generated
  size_t plan_size = global_path.poses.size();
  if (plan_size > 0) {
    global_path.poses.back().pose.orientation = goal.pose.orientation;
  }

  // If use_final_approach_orientation=true, interpolate the last pose orientation from the
  // previous pose to set the orientation to the 'final approach' orientation of the robot so
  // it does not rotate.
  // And deal with corner case of plan of length 1
  if (use_final_approach_orientation_) {
    if (plan_size == 1) {
      global_path.poses.back().pose.orientation = start.pose.orientation;
    } else if (plan_size > 1) {
      double dx, dy, theta;
      auto last_pose = global_path.poses.back().pose.position;
      auto approach_pose = global_path.poses[plan_size - 2].pose.position;
      dx = last_pose.x - approach_pose.x;
      dy = last_pose.y - approach_pose.y;
      theta = atan2(dy, dx);
      global_path.poses.back().pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }
  }

  auto stop_time = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
  RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
  RCLCPP_DEBUG(logger_, "the nodes_opened are:  %i", planner_->nodes_opened);
  return global_path;
}

void ThetaStarPlanner::getPlan(
  nav_msgs::msg::Path & global_path,
  std::function<bool()> cancel_checker)
{
  std::vector<coordsW> path;
  if (planner_->isUnsafeToPlan()) {
    global_path.poses.clear();
    throw nav2_core::PlannerException("Either of the start or goal pose are an obstacle! ");
  } else if (planner_->generatePath(path, cancel_checker)) {
    global_path = linearInterpolation(path, planner_->costmap_->getResolution());
  } else {
    global_path.poses.clear();
    throw nav2_core::NoValidPathCouldBeFound("Could not generate path between the given poses");
  }
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = global_frame_;
}

nav_msgs::msg::Path ThetaStarPlanner::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;

  geometry_msgs::msg::PoseStamped p1;
  for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
    coordsW pt1 = raw_path[j];
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    pa.poses.push_back(p1);

    coordsW pt2 = raw_path[j + 1];
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      pa.poses.push_back(p1);
    }
  }

  return pa;
}

rcl_interfaces::msg::SetParametersResult
ThetaStarPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".how_many_corners") {
        planner_->how_many_corners_ = parameter.as_int();
      }
      if (name == name_ + ".terminal_checking_interval") {
        planner_->terminal_checking_interval_ = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".w_euc_cost") {
        planner_->w_euc_cost_ = parameter.as_double();
      } else if (name == name_ + ".w_traversal_cost") {
        planner_->w_traversal_cost_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        planner_->allow_unknown_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_theta_star_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_theta_star_planner::ThetaStarPlanner, nav2_core::GlobalPlanner)
