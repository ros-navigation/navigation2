#ifndef NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
#define NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_

// Interfaces
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Others
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_localization/custom_particle_filter.hpp"

namespace nav2_localization
{
class DummySolver2d : public nav2_localization::Solver
{
public:
	DummySolver2d() {};

	geometry_msgs::msg::TransformStamped solve(
		const geometry_msgs::msg::TransformStamped& curr_odom);

	void configure(
		const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
		SampleMotionModel::Ptr& motionSampler,
		Matcher2d::Ptr& matcher,
		const geometry_msgs::msg::TransformStamped& odom,
		const geometry_msgs::msg::Pose& pose);

	void activate();
	void deactivate();
	void cleanup();

	void CreateParticleFilter(
		unsigned int NUM_SAMPLES,
		unsigned int STATE_SIZE,
		float PRIOR_MU_X,
		float PRIOR_MU_Y,
		float PRIOR_MU_THETA,
		float PRIOR_COV_X,
		float PRIOR_COV_Y,
		float PRIOR_COV_THETA);

private:
	CustomParticleFilter* pf_;
	std::make_shared<std::vector<geometry_msgs::msg::TransformStamped>> prior_samples_;
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
