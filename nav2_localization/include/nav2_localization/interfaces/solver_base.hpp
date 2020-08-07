#ifndef NAV2_LOCALIZATION__SOLVER_BASE_HPP_
#define NAV2_LOCALIZATION__SOLVER_BASE_HPP_

// Other Interfaces
#include "interfaces/motion_model_base.hpp"
#include "interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/poseWithCovariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/LaserScan.hpp"
#include "nav_msgs/msg/OccupancyGrid.hpp"

namespace nav2_localization_base
{

/**
 * @class Solver
 * @brief Abstract interface for solvers (such as Monte-Carlo Localization or Kalman Filter) to adhere to with pluginlib
 */
class Solver
{
public:
	using Ptr = std::shared_ptr<nav2_localization_base::Solver>;

	/**
     * @brief Estimates a pose fusing odometry and sensor information
     * @param motionSampler The way to carry uncertainty from the odometry to the pose estimation
     * @param matcher The technique that will be used to compute how likely it is to be in a cerain pose given the obtained measurement
     * @param prev_odom Previous pose odometry-based estimation
     * @param curr_odom Current pose odometry-based estimation
     * @param prev_pose Previous pose estimation
     * @param scan Current measurement
     * @param map Map of the environment where the robot is
     * @return Estimation of the current position
     */
	virtual PoseWithCovariance localize(
		const SampleMotionModel& motionSampler,
		const Matcher2d& matcher,
		const nav_msgs::msg::Odometry& prev_odom,
		const nav_msgs::msg::Odometry& curr_odom,
		const geometry_msgs::msg::Pose& prev_pose,
		const sensor_msgs::LaserScan& scan,
		const nav_msgs::OccupancyGrid& map) = 0;

protected:
	Solver(){}
};
} // nav2_localization_base

#endif // NAV2_LOCALIZATION__SOLVER_BASE_HPP_
