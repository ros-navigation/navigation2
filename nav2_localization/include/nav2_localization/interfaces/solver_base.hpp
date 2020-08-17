#ifndef NAV2_LOCALIZATION__SOLVER_BASE_HPP_
#define NAV2_LOCALIZATION__SOLVER_BASE_HPP_

// Other Interfaces
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Others
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{

/**
 * @class Solver
 * @brief Abstract interface for solvers (such as Monte-Carlo Localization or Kalman Filter) to adhere to with pluginlib
 */
class Solver
{
public:
	Solver(){}

	using Ptr = std::shared_ptr<nav2_localization::Solver>;

	/**
     * @brief Estimates a pose fusing odometry and sensor information
     * @param curr_odom Current pose odometry-based estimation
     * @param scan Current measurement
     * @return Estimation of the current position
     */
	virtual geometry_msgs::PoseWithCovariance solve(
		const nav_msgs::msg::Odometry& curr_odom,
		const sensor_msgs::msg::LaserScan& scan) = 0;

	virtual void configure(
		const rclcpp_lifecycle::LifecycleNode::SharedPtr& init_node,
		SampleMotionModel* init_motionSampler,
		Matcher2d* init_matcher,
		nav_msgs::msg::Odometry init_odom,
		geometry_msgs::msg::Pose init_pose) = 0;

	virtual void activate() = 0;

	virtual void deactivate() = 0;

	virtual void cleanup() = 0;

private:
	rclcpp_lifecycle::LifecycleNode::SharedPtr node;
	SampleMotionModel* motionSampler; // Pointer to the MotionSampler (will be used to carry uncertainty from the odom to the pose estimation)
	Matcher2d* matcher; // Pointer to the Matcher (Will be used to compute how likely it is to be in a certain pose given the obtained measurement)
	nav_msgs::msg::Odometry prev_odom; // Previous pose odometry-based estimation
	geometry_msgs::msg::Pose prev_pose; // Previous pose estimation
	
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__SOLVER_BASE_HPP_
