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

	BFL::Probability ProbabilityGet(const sensor_msgs::msg::LaserScan &measurement) const;

	void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

	void setMap(const nav_msgs::msg::OccupancyGrid& map);
};  
} // nav2_localization

#endif // NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_
