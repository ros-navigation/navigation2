#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include "nav2_localization/plugins/matcher2d_plugins.hpp"

namespace nav2_localization
{

float DummyMatcher2d::match(
	const sensor_msgs::msg::LaserScan& scan,
	const geometry_msgs::msg::Pose& pose)
{
	float probability = 0.5;
	return probability;
}

void DummyMatcher2d::configure(
	const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
	nav_msgs::msg::OccupancyGrid* map)
{
	node_ = node;
	map_ = map;
	return;
}

void DummyMatcher2d::activate()
{

}

void DummyMatcher2d::deactivate()
{

}

void DummyMatcher2d::cleanup()
{

}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DummyMatcher2d, nav2_localization::Matcher2d)
