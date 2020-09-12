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
		const nav_msgs::msg::Odometry& curr_odom) = 0;

	virtual void configure(
		const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
		SampleMotionModel& motionSampler,
		Matcher2d& matcher,
		const nav_msgs::msg::Odometry& odom,
		const geometry_msgs::msg::Pose& pose) = 0;

	virtual void activate() = 0;

	virtual void deactivate() = 0;

	virtual void cleanup() = 0;

private:
	rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
	SampleMotionModel& motionSampler_; // Reference to the MotionSampler (will be used to carry uncertainty from the odom to the pose estimation)
	Matcher2d& matcher_; // Reference to the Matcher (Will be used to compute how likely it is to be in a certain pose given the obtained measurement)
	nav_msgs::msg::Odometry prev_odom_; // Previous pose odometry-based estimation
	geometry_msgs::msg::Pose prev_pose_; // Previous pose estimation
	
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__SOLVER_BASE_HPP_
