#ifndef NAV2_LOCALIZATION__SOLVER_BASE_HPP_
#define NAV2_LOCALIZATION__SOLVER_BASE_HPP_

// Other Interfaces
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
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
		const geometry_msgs::msg::TransformStamped& curr_odom,
		const sensor_msgs::msg::Pointcloud2::ConstSharedPtr& scan) = 0;

	/**
	 * @brief Initializes the filter being used with a given pose
	 * @param pose The pose at which to initialize the filter
	 */ 
	virtual void initFilter(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &pose) = 0;

	/**
     * @brief Configures the solver, during the "Configuring" state of the parent lifecycle node.
     * @param node Pointer to the parent lifecycle node
	 * @param motionSampler The Sample Motion Model to use 
	 * @param matcher The 2D Matcher to use
	 * @param odom Initial odometry
	 * @param pose Initial pose
     */   
	virtual void configure(
		const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
		SampleMotionModel::Ptr& motionSampler,
		Matcher2d::Ptr& matcher,
		const geometry_msgs::msg::TransformStamped& odom,
		const geometry_msgs::msg::TransformStamped& pose) = 0;

	/**
     * @brief Activates the solver, during the "Activating" state of the parent lifecycle node.
     */
	virtual void activate() = 0;

	/**
     * @brief Deactivates the solver, during the "Deactivating" state of the parent lifecycle node. 
     */
	virtual void deactivate() = 0;

	/**
     * @brief Cleans up the solver, during the "Cleaningup" state of the parent lifecycle node.
     */
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
