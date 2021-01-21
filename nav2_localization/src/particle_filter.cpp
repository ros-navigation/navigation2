#include "nav2_localization/particle_filter.hpp"

namespace  nav2_localization
{
ParticleFilter::ParticleFilter(const int &initial_number_of_particles,
								const geometry_msgs::msg::TransformStamped &initial_pose)
{}

void ParticleFilter::prediction_step(const SampleMotionModel::Ptr &motion_sampler,
										 const geometry_msgs::msg::TransformStamped& prev_odom,
										 const geometry_msgs::msg::TransformStamped& curr_odom,
										 const geometry_msgs::msg::TransformStamped& prev_pose)
{}

void ParticleFilter::update_step(const Matcher2d::Ptr &matcher2d,
									  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan)
{}

geometry_msgs::msg::TransformStamped ParticleFilter::get_most_likely_pose()
{
	return geometry_msgs::msg::TransformStamped();
}
} // nav2_localization
