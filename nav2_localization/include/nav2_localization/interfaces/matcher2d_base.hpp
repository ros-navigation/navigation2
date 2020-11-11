#ifndef NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
#define NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_localization
{

/**
 * @class Matcher2d
 * @brief Abstract interface for a 2D matcher for localization purposes to adhere to with pluginlib
 */

class Matcher2d
{
public:
    Matcher2d(){}

	using Ptr = std::shared_ptr<nav2_localization::Matcher2d>;

	virtual void configure(const nav2_util::LifecycleNode::SharedPtr& node) = 0;

	virtual void activate() = 0;

	virtual void deactivate() = 0;

	virtual void cleanup() = 0;

	virtual void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map) = 0;

	virtual void setLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan) = 0;

	virtual sensor_msgs::msg::LaserScan::ConstSharedPtr getLaserScan() = 0;

	virtual void setLaserPose(const geometry_msgs::msg::TransformStamped& laser_pose) = 0;

protected:
	nav2_util::LifecycleNode::SharedPtr node_;
	nav_msgs::msg::OccupancyGrid::SharedPtr map_; // Reference to the 2D occupancy grid map of the environment where the robot is
	sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan_;
	geometry_msgs::msg::TransformStamped laser_pose_;
};  
} // nav2_localization

#endif // NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
