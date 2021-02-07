// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

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
	void initFilter(const int &initial_number_of_particles, const geometry_msgs::msg::TransformStamped &init_pose);
	void update(const geometry_msgs::msg::TransformStamped &prev_odom,
				const geometry_msgs::msg::TransformStamped &curr_odom,
				const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan,
				const SampleMotionModel::Ptr &motion_model,
				const Matcher2d::Ptr &matcher);
	geometry_msgs::msg::TransformStamped getMostLikelyPose();
	std::vector<Particle> getParticles();

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
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_
