#include <stdint.h>
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "mppic/controller.hpp"
#include "mppic/motion_models.hpp"
#include "mppic/utils.hpp"

namespace mppi
{

void Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;

  auto node = parent_.lock();
  // Get high-level controller parameters
  auto getParam = utils::getParamGetter(node, name_);
  getParam(visualize_, "visualize", false);

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_);
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_);
  trajectory_visualizer_.on_configure(parent_, costmap_ros_->getGlobalFrameID());

  auto vels = optimizer_.getControlConstraints();
  base_x_vel_ = vels.vx;
  base_y_vel_ = vels.vy;
  base_theta_vel_ = vels.vw;

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void Controller::cleanup()
{
  trajectory_visualizer_.on_cleanup();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void Controller::activate()
{
  trajectory_visualizer_.on_activate();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void Controller::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);
  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);

  visualize(robot_pose, robot_speed, std::move(transformed_plan));
  return cmd;
}

void Controller::visualize(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav_msgs::msg::Path transformed_plan)
{
  if (!visualize_) {
    return;
  }

  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), 5, 2);
  trajectory_visualizer_.add(optimizer_.evalTrajectoryFromControlSequence(robot_pose, robot_speed));
  trajectory_visualizer_.visualize(transformed_plan);
}

void Controller::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void Controller::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  utils::ControlConstraints constraints;

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    constraints = utils::ControlConstraints{base_x_vel_, base_y_vel_, base_theta_vel_};
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      constraints.vx = base_x_vel_ * speed_limit / 100.0;
      constraints.vy = base_y_vel_ * speed_limit / 100.0;
      constraints.vw = base_theta_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / base_x_vel_;
      constraints.vx = speed_limit;
      constraints.vy = base_y_vel_ * ratio;
      constraints.vw = base_theta_vel_ * ratio;
    }
  }

  optimizer_.setControlConstraints(constraints);
}

} // namespace mppi

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mppi::Controller, nav2_core::Controller)
