#ifndef NAV2_LOCALIZATION__SOLVER_BASE_HPP_
#define NAV2_LOCALIZATION__SOLVER_BASE_HPP_

// Other Interfaces
#include "nav2_localization/interfaces/motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_Scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

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
     * @param curr_odom Current pose odometry-based estimation
     * @param scan Current measurement
     * @param map Map of the environment where the robot is
     * @return Estimation of the current position
     */
	virtual geometry_msgs::PoseWithCovariance localize(
		const SampleMotionModel& motionSampler,
		const Matcher2d& matcher,
		const nav_msgs::msg::Odometry& curr_odom,
		const sensor_msgs::msg::LaserScan& scan,
		const nav_msgs::msg::OccupancyGrid& map) = 0;

	virtual void configure(
		const nav_msgs::msg::Odometry& init_odom=nav_msgs::msg::Odometry{},
		const geometry_msgs::msg::Pose& init_pose=geometry_msgs::msg::Pose{}) = 0;

protected:
	Solver(){}

private:
	nav_msgs::msg::Odometry prev_odom; // Previous pose odometry-based estimation
	geometry_msgs::msg::Pose prev_pose; // Previous pose estimation
	
};
} // nav2_localization_base

#endif // NAV2_LOCALIZATION__SOLVER_BASE_HPP_
