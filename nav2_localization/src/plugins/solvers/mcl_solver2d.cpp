// Copyright (c) 2021 Jose M. TORRES-CAMARA and Khaled SAAD

#include <vector>
#include <memory>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solvers/mcl_solver2d.hpp"
#include "nav2_localization/particle_filter.hpp"

namespace nav2_localization
{
geometry_msgs::msg::TransformStamped MCLSolver2d::solve(
  const geometry_msgs::msg::TransformStamped& curr_odom,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan)
{
  // Motion update
  pf_->update();
  prev_odom_ = curr_odom;

  // Measurement update
  pf_->resample();

  geometry_msgs::msg::TransformStamped curr_pose;
  // curr_pose = pf_->get_most_likely_pose();

  prev_pose_ = curr_pose;

    return curr_pose;
}

void MCLSolver2d::initFilter(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &pose)
{}

void MCLSolver2d::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  SampleMotionModel::Ptr& motionSampler,
  Matcher2d::Ptr& matcher,
  const geometry_msgs::msg::TransformStamped& odom,
  const geometry_msgs::msg::TransformStamped& pose)
{
  node_ = node;

  node_->declare_parameter("num_particles", 1000);

  motionSampler_ = motionSampler;
  matcher_ = matcher;

  // Get configuration and generate PF
  int number_of_particles;
  node_->get_parameter("num_particles", number_of_particles);

  pf_ = std::make_shared<ParticleFilter>(number_of_particles);

  prev_odom_ = odom;
  prev_pose_ = pose;
  return;
}

void MCLSolver2d::activate()
{}

void MCLSolver2d::deactivate()
{}

void MCLSolver2d::cleanup()
{}

}  // namespace nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::MCLSolver2d, nav2_localization::Solver)
