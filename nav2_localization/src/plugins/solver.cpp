#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solver_plugins.hpp"

namespace nav2_localization
{

geometry_msgs::msg::PoseWithCovariance DummySolver2d::localize(
	const nav2_localization::SampleMotionModel& motionSampler,
	const nav2_localization::Matcher2d& matcher,
	const nav_msgs::msg::Odometry& curr_odom,
	const sensor_msgs::msg::LaserScan& scan,
	const nav_msgs::msg::OccupancyGrid& map)
{
    geometry_msgs::msg::PoseWithCovariance dummy_pose{};
    return dummy_pose;
}

void DummySolver2d::configure(
	const nav_msgs::msg::Odometry& init_odom=nav_msgs::msg::Odometry{},
	const geometry_msgs::msg::Pose& init_pose=geometry_msgs::msg::Pose{})
{
	// Initialize member variables
	// Default odometry and pose constructors are used by default
	prev_odom = init_odom;
	prev_pose = init_pose;
	return
}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DummySolver2d, nav2_localization::Solver)
