#include "angles/angles.h"

#include "tf2/utils.h"

#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/plugins/solvers/mcl_solver.hpp"

namespace nav2_localization
{
MCLSolver::MCLSolver() {}

geometry_msgs::msg::PoseWithCovarianceStamped MCLSolver::estimatePose(
  const nav_msgs::msg::Odometry & curr_odom,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan)
{
  RCLCPP_DEBUG(node_->get_logger(), "Filtering...");

  // TODO(marwan99): Add more resampling throttling methods e.g. frequency.
  // TODO(marwan99): Make dist threshold a parameter
  // TODO(marwan99): Fix resampling condition
  bool do_resmaple = true;  // Set to false to enable throtelling
  // double delta_x = prev_odom_.pose.pose.position.x - curr_odom.pose.pose.position.x;
  // double delta_y = prev_odom_.pose.pose.position.y - curr_odom.pose.pose.position.y;
  // double delta_yaw = abs(
  //   tf2::getYaw(prev_odom_.pose.pose.orientation) -
  //   tf2::getYaw(curr_odom.pose.pose.orientation));
  // if (delta_x * delta_x + delta_y * delta_y > 0.01 || delta_yaw > 0.1) {
  //   do_resmaple = true;
  // }

  weights_sum_ = 0;

  for (auto & particle : particles_) {
    particle.pose_ = motion_sampler_->getMostLikelyPose(prev_odom_, curr_odom, particle.pose_);

    if (do_resmaple) {
      particle.weight_ = matcher_->getScanProbability(scan, particle.pose_);
      weights_sum_ += particle.weight_;
    }
  }

  // Normalise the weights of the particles
  for (auto & particle : particles_) {
    particle.weight_ /= weights_sum_;
  }

  if (do_resmaple) {
    resample();
  } else {
    return prev_pose_;
  }

  visualize_particles();

  // Get mean pose from the distribution respresented by the particle set
  auto estimated_pose = getMeanPose();

  prev_pose_ = estimated_pose;
  prev_odom_ = curr_odom;

  return estimated_pose;
}

}  // namespace nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::MCLSolver, nav2_localization::Solver)
