#ifndef NAV2_LOCALIZATION__MCL_SOLVER2D_HPP_
#define NAV2_LOCALIZATION__MCL_SOLVER2D_HPP_

// Interfaces
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan) override;

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
	void publishParticleCloud();

	std::shared_ptr<ParticleFilter> pf_; // Particle filter
	int init_number_of_particles_;
	bool first_iteration_;
	double motion_linear_tol_;
	double motion_angular_tol_;
	rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
	std::string map_frame_id_;
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__MCL_SOLVER2D_HPP_