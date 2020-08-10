#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include "nav2_localization/plugins/matcher2d_plugins.hpp"

namespace nav2_localization
{

float DummyMatcher2d::getLikelihood(
	const sensor_msgs::msg::LaserScan& scan,
	const geometry_msgs::msg::Pose& pose,
	const nav_msgs::msg::OccupancyGrid& map)
{
	float probability = 0.5;
	return probability;
}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DummyMatcher2d, nav2_localization::Matcher2d)
