#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solver_plugins.hpp"

namespace nav2_localization
{

geometry_msgs::msg::PoseWithCovariance DummySolver2d::solve(
	const nav_msgs::msg::Odometry& curr_odom,
	const sensor_msgs::msg::LaserScan& scan)
{
    geometry_msgs::msg::PoseWithCovariance dummy_pose{};
    return dummy_pose;
}

void DummySolver2d::configure(
	const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
	SampleMotionModel* motionSampler,
	Matcher2d* matcher,
	nav_msgs::msg::Odometry odom,
	geometry_msgs::msg::Pose pose)
{
	node_ = node;
	motionSampler_ = motionSampler;
	matcher_ = matcher
	prev_odom_ = odom;
	prev_pose_ = pose;
	return;
}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DummySolver2d, nav2_localization::Solver)
