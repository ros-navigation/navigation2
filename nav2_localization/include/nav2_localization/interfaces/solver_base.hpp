#ifndef NAV2_LOCALIZATION__SOLVER_BASE_HPP_
#define NAV2_LOCALIZATION__SOLVER_BASE_HPP_

// Other Interfaces
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
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
	virtual geometry_msgs::msg::TransformStamped solve(
		const geometry_msgs::msg::TransformStamped& curr_odom) = 0;

	virtual void init_filter(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &pose) = 0;

	virtual void configure(
		const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
		SampleMotionModel::Ptr& motionSampler,
		Matcher2d::Ptr& matcher,
		const geometry_msgs::msg::TransformStamped& odom,
		const geometry_msgs::msg::TransformStamped& pose) = 0;

	virtual void activate() = 0;

	virtual void deactivate() = 0;

	virtual void cleanup() = 0;

protected:
	rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
	SampleMotionModel::Ptr motionSampler_;
	Matcher2d::Ptr matcher_;
	geometry_msgs::msg::TransformStamped prev_odom_; // Previous pose odometry-based estimation
	geometry_msgs::msg::TransformStamped prev_pose_; // Previous pose estimation
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__SOLVER_BASE_HPP_
