#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solver_plugins.hpp"
#include "nav2_localization/dummy_particle_filter.hpp"
#include <vector>

namespace nav2_localization
{
geometry_msgs::msg::TransformStamped DummySolver2d::solve(
	const geometry_msgs::msg::TransformStamped& curr_odom,
	const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan)
{
	// Motion update
	pf_->prediction_step(motionSampler_, curr_odom, prev_odom_, prev_pose_);
	prev_odom_ = curr_odom;

	// Measurement update
	pf_->update_step(matcher_, laser_scan);

	geometry_msgs::msg::TransformStamped curr_pose = pf_->get_most_likely_pose();
	prev_pose_ = curr_pose;

    return curr_pose;
}

void DummySolver2d::init_filter(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &pose)
{

}

void DummySolver2d::configure(
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

	pf_ = std::make_shared<DummyParticleFilter>(number_of_particles, pose);

	prev_odom_ = odom;
	prev_pose_ = pose;
	return;
}

void DummySolver2d::activate()
{

}

void DummySolver2d::deactivate()
{

}

void DummySolver2d::cleanup()
{

}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DummySolver2d, nav2_localization::Solver)
