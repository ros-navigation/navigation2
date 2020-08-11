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
	SampleMotionModel* init_motionSampler,
	Matcher2d* init_matcher,
	nav_msgs::msg::Odometry init_odom,
	geometry_msgs::msg::Pose init_pose)
{
	motionSampler = init_motionSampler;
	matcher = init_matcher
	prev_odom = init_odom;
	prev_pose = init_pose;
	return
}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DummySolver2d, nav2_localization::Solver)
