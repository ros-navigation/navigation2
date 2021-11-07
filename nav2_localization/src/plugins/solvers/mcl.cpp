
#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/plugins/solvers/mcl.hpp"

namespace nav2_localization
{
MCL::MCL() {}

geometry_msgs::msg::PoseWithCovarianceStamped MCL::estimatePose(
  const nav_msgs::msg::Odometry & curr_odom,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan)
{
  weights_sum_ = 0;
 
  for (auto & particle : particles_) {
    particle.pose_ = motion_sampler_->getMostLikelyPose(prev_odom_, curr_odom, particle.pose_);
    particle.weight_ = matcher_->getScanProbability(scan, particle.pose_);

    weights_sum_ += particle.weight_;
  }

  // Normalise the weights of the particles
  for (auto & particle : particles_)
    particle.weight_ /= weights_sum_;

  resample();
  visualize_particles();

  // Save current odometry
  prev_odom_.pose = curr_odom.pose;

  // Get mean pose from the distribution respresented by the particle set
  auto estimated_pose = getMeanPose();
  estimated_pose.header = curr_odom.header;

  return estimated_pose;
}

}  // namespace nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::MCL, nav2_localization::Solver)