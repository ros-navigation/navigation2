#ifndef NAV2_LOCALIZATION__DUMMY_PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__DUMMY_PARTICLE_FILTER_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include <vector>

namespace nav2_localization
{
struct particle
{
	geometry_msgs::msg::TransformStamped pose;
	float weight;
};

class ParticleFilter
{
public:
	ParticleFilter(const int &initial_number_of_particles,
					const geometry_msgs::msg::TransformStamped &initial_pose);
	void prediction_step(const SampleMotionModel::Ptr &motion_sampler,
						 const geometry_msgs::msg::TransformStamped& prev_odom,
        				 const geometry_msgs::msg::TransformStamped& curr_odom,
						 const geometry_msgs::msg::TransformStamped& prev_pose);
	void update_step(const Matcher2d::Ptr &matcher2d,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan);
	geometry_msgs::msg::TransformStamped get_most_likely_pose();

private:
	std::vector<particle> particles_;
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__DUMMY_PARTICLE_FILTER_HPP_
