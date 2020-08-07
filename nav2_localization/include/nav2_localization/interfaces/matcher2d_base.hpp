#ifndef NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
#define NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/LaserScan.hpp"
#include "nav_msgs/msg/OccupancyGrid.hpp"

namespace nav2_localization_base
{

/**
 * @class Matcher2d
 * @brief Abstract interface for a 2D matcher for localization purposes to adhere to with pluginlib
 */
class Matcher2d
{
public:
	using Ptr = std::shared_ptr<nav2_localization_base::Matcher2d>;

    /**
     * @brief Computes how likely is it to read a measurement given a position in a map
     * @param scan 2D laser-like measurement
     * @param pose Hypothesis on the robot's pose
     * @param map 2D occupancy grid map of the environment where the robot is
     * @return Score of how likely is it to perceive the inputted measurement assuming that the robot is in the given pose inside the provided map
     */
	virtual float getLikelihood(
		const sensor_msgs::LaserScan& scan,
		const geometry_msgs::msg::Pose& pose,
		const nav_msgs::OccupancyGrid& map) = -1;

protected:
    Matcher2d(){}
};  
} // nav2_localization_base

#endif // NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
