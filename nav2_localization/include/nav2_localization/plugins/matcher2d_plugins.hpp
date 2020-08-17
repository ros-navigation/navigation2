#ifndef NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_
#define NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_

#include <nav2_localization/interfaces/matcher2D_base.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{
class DummyMatcher2d : public nav2_localization::Matcher2d
{
public:
	DummyMatcher2d(){}

	float match(
		const sensor_msgs::msg::LaserScan& scan,
		const geometry_msgs::msg::Pose& pose);

	void configure(
		const rclcpp_lifecycle::LifecycleNode::SharedPtr& init_node,
		nav_msgs::msg::OccupancyGrid* init_map);
};  
} // nav2_localization

#endif // NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_
