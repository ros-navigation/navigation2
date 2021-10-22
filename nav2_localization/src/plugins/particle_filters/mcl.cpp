// Copyright (c) 2021 Marwan TAHER and Khaled SAAD
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

#include <vector>
#include <random>

#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"

#include "nav2_localization/plugins/particle_filters/mcl.hpp"

namespace nav2_localization
{
MCL::MCL()
: ParticleFilter() {}

nav_msgs::msg::Odometry MCL::estimatePose(
  const nav_msgs::msg::Odometry & curr_odom,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan)
{
  for (auto & particle : particles_) {
    // TODO(unassigned): Sort out types incompatibility
    particle.pose = motion_sampler_->getMostLikelyPose(prev_estimate_, curr_odom, particle.pose);
    particle.weight_ = matcher_->getScanProbability(scan, particle.pose);
  }

  lowVarianceResample();

  auto estimated_pose = getMeanPose();
  prev_estimate_ = estimated_pose;

  return estimated_pose;
}

void MCL::lowVarianceResample()
{
  std::vector<Particle> sampled_particles;

  std::random_device rand_device;
  std::mt19937 gen(rand_device());
  std::uniform_real_distribution<> dist;

  double c = particles_[0].weight_;

  for (int i = 0; i < particles_count_; i++) {
    double u = dist(gen) + i / particles_count_;
    while (u > c) {
      i++;
      c += particles_[i].weight_;
    }
    sampled_particles.push_back(particles_[i]);
  }

  particles_ = sampled_particles;
}

nav_msgs::msg::Odometry MCL::getMeanPose()
{
  double mean_x = 0, mean_y = 0, mean_yaw = 0;

  for (auto particle : particles_) {
    mean_x += particle.weight_ * particle.pose_.position.x;
    mean_y += particle.weight_ * particle.pose_.position.y;

    double yaw = tf2::getYaw(particle.pose_.orientation);
    mean_yaw += particle.weight_ * yaw;
  }

  mean_x /= particles_count_;
  mean_y /= particles_count_;
  mean_yaw /= particles_count_;

  double cov_x = 0, cov_y = 0, cov_yaw = 0;
  for (auto particle : particles_) {
    cov_x += (particle.pose_.position.x - mean_x) * (particle.pose_.position.x - mean_x);
    cov_y += (particle.pose_.position.y - mean_y) * (particle.pose_.position.y - mean_y);

    double yaw = tf2::getYaw(particle.pose_.orientation);
    cov_yaw += (yaw - mean_y) * (yaw - mean_y);
  }

  cov_x /= particles_count_;
  cov_y /= particles_count_;
  cov_yaw /= particles_count_;

  nav_msgs::msg::Odometry estimated_odom;

  estimated_odom.header.frame_id = prev_estimate_.header.frame_id;
  estimated_odom.header.stamp = rclcpp::Time();

  estimated_odom.pose.pose.position.x = mean_x;
  estimated_odom.pose.pose.position.y = mean_y;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, mean_yaw);
  estimated_odom.pose.pose.orientation = tf2::toMsg(quat);

  estimated_odom.pose.covariance = {cov_x, 0, 0, 0, 0, 0,
    0, cov_y, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, cov_yaw};

  return estimated_odom;
}
}  // namespace nav2_localization
