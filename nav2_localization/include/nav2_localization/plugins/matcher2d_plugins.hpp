#ifndef NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_
#define NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_

#include <nav2_localization/interfaces/matcher2D_base.hpp>

namespace nav2_localization_plugins
{
class DummyMatcher2d : public nav2_localization_base::Matcher2d
{
public:
	DummyMatcher2d(){}

	float getLikelihood(
		const sensor_msgs::LaserScan& scan,
		const geometry_msgs::msg::Pose& pose,
		const nav_msgs::OccupancyGrid& map);
};  
} // nav2_localization_base

#endif // NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_
