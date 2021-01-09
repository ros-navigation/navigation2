#ifndef NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
#define NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_

// Interfaces
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Others
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_localization/particle_filter.hpp"

namespace nav2_localization
{
class MCLSolver2d : public nav2_localization::Solver
{
public:
	MCLSolver2d() {};

	geometry_msgs::msg::TransformStamped solve(
		const geometry_msgs::msg::TransformStamped& curr_odom,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan) override;

	void initFilter(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &pose) override;

	void configure(
		const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
		SampleMotionModel::Ptr& motionSampler,
		Matcher2d::Ptr& matcher,
		const geometry_msgs::msg::TransformStamped& odom,
		const geometry_msgs::msg::TransformStamped& pose) override;

	void activate() override;
	void deactivate() override;
	void cleanup() override;

private:
	std::shared_ptr<ParticleFilter> pf_; // Particle filter
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
