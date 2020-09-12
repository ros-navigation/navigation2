#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include "nav2_localization/plugins/matcher2d_plugins.hpp"

namespace nav2_localization
{

void LikelihoodFieldMatcher2d::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
{
	node_ = node;
	return;
}

BFL::Probability LikelihoodFieldMatcher2d::ProbabilityGet(
	const sensor_msgs::msg::LaserScan &measurement) const
{
	// TODO: main logic goes here!
}

void LikelihoodFieldMatcher2d::activate()
{

}

void LikelihoodFieldMatcher2d::deactivate()
{

}

void LikelihoodFieldMatcher2d::cleanup()
{

}

void LikelihoodFieldMatcher2d::setMap(const nav_msgs::msg::OccupancyGrid& map)
{
	map_ = map;
}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::LikelihoodFieldMatcher2d, nav2_localization::Matcher2d)
