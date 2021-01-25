
#ifndef NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_

#include <vector>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "interfaces/sample_motion_model_base.hpp"
#include "interfaces/matcher2d_base.hpp"

namespace nav2_localization
{
class Particle
{
public:
	Particle(const geometry_msgs::msg::TransformStamped &pose, const double &weight) : pose_(pose), weight_(weight) {}
	geometry_msgs::msg::TransformStamped pose_;
	double weight_;
};

class ParticleFilter
{
public:
	ParticleFilter(const int &initial_number_of_particles, const geometry_msgs::msg::TransformStamped &init_pose);
	void update(const geometry_msgs::msg::TransformStamped &prev_odom,
				const geometry_msgs::msg::TransformStamped &curr_odom,
				const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan,
				const SampleMotionModel::Ptr &motion_model,
				const Matcher2d::Ptr &matcher);
	geometry_msgs::msg::TransformStamped getMostLikelyPose();

private:
	std::vector<Particle> sample(const geometry_msgs::msg::TransformStamped &prev_odom,
								 const geometry_msgs::msg::TransformStamped &curr_odom,
								 const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan,
								 const SampleMotionModel::Ptr &motion_model,
								 const Matcher2d::Ptr &matcher);
	std::vector<Particle> resample(const std::vector<Particle> &particles_t_bar);
	void updateStats();

	double sum_of_weights_;
	std::vector<Particle> particles_t_;
	std::vector<Particle> particles_t_1_;
	geometry_msgs::msg::TransformStamped most_likely_pose_;
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_