#ifndef NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
#define NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <pdf/conditionalpdf.h>

#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1
#define MEASMODEL_DIMENSION_MOBILE        1

namespace nav2_localization
{

/**
 * @class Matcher2d
 * @brief Abstract interface for a 2D matcher for localization purposes to adhere to with pluginlib
 */
class Matcher2d : public BFL::ConditionalPdf
	<sensor_msgs::msg::LaserScan, geometry_msgs::msg::TransformStamped>(MEASMODEL_DIMENSION_MOBILE,MEASMODEL_NUMCONDARGUMENTS_MOBILE)
{
public:
    Matcher2d(){}

	using Ptr = std::shared_ptr<nav2_localization::Matcher2d>;

	virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) = 0;

	virtual void activate() = 0;

	virtual void deactivate() = 0;

	virtual void cleanup() = 0;

	virtual void setMap(const nav_msgs::msg::OccupancyGrid& map) = 0;

protected:
	rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
	nav_msgs::msg::OccupancyGrid& map_; // Reference to the 2D occupancy grid map of the environment where the robot is
};  
} // nav2_localization

#endif // NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
