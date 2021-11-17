// Copyright (c) 2021 Marwan Taher and Khaled SAAD
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

#define _USE_MATH_DEFINES
#include <math.h>

#include <random>
#include <vector>

#include "angles/angles.h"

#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

#include "nav2_msgs/msg/particle.hpp"

#include "nav2_localization/plugins/solvers/particle_filter_base.hpp"

namespace nav2_localization
{
ParticleFilterSolver::ParticleFilterSolver() {}

void ParticleFilterSolver::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  SampleMotionModel::Ptr & motionSampler,
  Matcher2d::Ptr & matcher)
{
  Solver::configure(node, motionSampler, matcher);

  // TODO(marwan99): Remove particles_poses_pub_ once merged with upstream main
  particle_cloud_pub_ = node_->create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud", rclcpp::SensorDataQoS());

  particles_poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
    "particles_poses", rclcpp::SensorDataQoS());

  node_->declare_parameter("particles_count", 500);
  node_->declare_parameter("particles_init_radius_sd", 0.2);
  node_->declare_parameter("particles_init_yaw_sd", 0.2);

  node_->get_parameter("particles_count", particles_count_);
  node_->get_parameter("particles_init_radius_sd", particles_init_radius_sd_);
  node_->get_parameter("particles_init_yaw_sd", particles_init_yaw_sd_);
}

void ParticleFilterSolver::activate()
{
  particle_cloud_pub_->on_activate();
  particles_poses_pub_->on_activate();
}

void ParticleFilterSolver::initPose(const geometry_msgs::msg::PoseWithCovarianceStamped & init_pose)
{
  Solver::initPose(init_pose);

  // Initialize particles randomly around init_pose within a radius particles_init_radius_
  geometry_msgs::msg::Pose temp_pose;

  for (int i = 0; i < particles_count_; i++) {
    std::random_device rand_device;
    std::mt19937 gen(rand_device());
    std::normal_distribution<double> pose_dist(0, particles_init_radius_sd_);
    std::normal_distribution<double> yaw_dist(0, particles_init_yaw_sd_);

    temp_pose.position.x = init_pose.pose.pose.position.x + pose_dist(gen);
    temp_pose.position.y = init_pose.pose.pose.position.y + pose_dist(gen);

    double yaw = tf2::getYaw(init_pose.pose.pose.orientation) + yaw_dist(gen);

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    temp_pose.orientation = tf2::toMsg(quat);

    particles_.push_back(Particle(temp_pose, 1));
  }

  visualize_particles();
}

// TODO(all): Discuss where resample should live
// Current suggestions are ParticleFilterSolver, MCLSolver or as plugin.
void ParticleFilterSolver::resample()
{
  std::vector<Particle> sampled_particles;

  double particle_count_dtype = static_cast<double>(particles_count_);

  std::random_device rand_device;
  std::mt19937 gen(rand_device());
  std::uniform_real_distribution<double> dist(0, 1 / particle_count_dtype);

  double weights_sum = particles_[0].weight_;
  double rand_seed = dist(gen);

  int i = 0;
  for (int m = 0; m < particles_count_; m++) {
    double u = rand_seed + (m / particle_count_dtype);
    while (u > weights_sum) {
      i++;
      weights_sum += particles_[i].weight_;
    }
    sampled_particles.push_back(particles_[i]);
  }

  particles_ = sampled_particles;
}

void ParticleFilterSolver::visualize_particles()
{
  // TODO(marwan99): Pass global frame id to solvers
  nav2_msgs::msg::ParticleCloud particle_cloud_msg;
  particle_cloud_msg.header.frame_id = "map";
  particle_cloud_msg.header.stamp = node_->now();

  geometry_msgs::msg::PoseArray particles_poses_msg;
  particles_poses_msg.header.frame_id = "map";
  particles_poses_msg.header.stamp = node_->now();

  for (const auto & particle : particles_) {
    particles_poses_msg.poses.push_back(particle.pose_);

    nav2_msgs::msg::Particle temp_particle;
    temp_particle.pose = particle.pose_;
    temp_particle.weight = particle.weight_;
    particle_cloud_msg.particles.push_back(temp_particle);
  }

  particles_poses_pub_->publish(particles_poses_msg);
  particle_cloud_pub_->publish(particle_cloud_msg);
}

// TODO(marwan99): Expand to work for 3D as well.
geometry_msgs::msg::PoseWithCovarianceStamped ParticleFilterSolver::getMeanPose()
{
  // Calculate mean of the particle distribution
  // Angles mean is calculated using circular mean
  // https://en.wikipedia.org/wiki/Circular_mean
  double mean_x = 0, mean_y = 0;
  double mean_yaw_sin = 0, mean_yaw_cos = 0;
  double weights_sum = 0;

  std::vector<double> yaw_sin, yaw_cos;

  for (auto particle : particles_) {
    mean_x += particle.weight_ * particle.pose_.position.x;
    mean_y += particle.weight_ * particle.pose_.position.y;

    double yaw = tf2::getYaw(particle.pose_.orientation);
    yaw_sin.push_back(std::sin(yaw));
    yaw_cos.push_back(std::cos(yaw));
    mean_yaw_sin += particle.weight_ * yaw_sin.back();
    mean_yaw_cos += particle.weight_ * yaw_cos.back();

    weights_sum += particle.weight_;
  }

  mean_x /= weights_sum;
  mean_y /= weights_sum;

  mean_yaw_sin /= weights_sum;
  mean_yaw_cos /= weights_sum;

  // Calculate covariance of the particle distribution
  double cov_x = 0, cov_y = 0, cov_yaw_sin = 0, cov_yaw_cos = 0;
  for (int i = 0; i < particles_.size(); i++) {
    cov_x += (particles_[i].pose_.position.x - mean_x) * (particles_[i].pose_.position.x - mean_x);
    cov_y += (particles_[i].pose_.position.y - mean_y) * (particles_[i].pose_.position.y - mean_y);

    cov_yaw_sin += (yaw_sin[i] - mean_yaw_sin) * (yaw_sin[i] - mean_yaw_sin);
    cov_yaw_cos += (yaw_cos[i] - mean_yaw_cos) * (yaw_cos[i] - mean_yaw_cos);
  }

  cov_x /= particles_count_;
  cov_y /= particles_count_;

  cov_yaw_sin /= particles_count_;
  cov_yaw_cos /= particles_count_;

  // Circular variance calculation
  // https://en.wikipedia.org/wiki/Directional_statistics#Measures_of_location_and_spread
  double cov_yaw = 1 - sqrt(cov_yaw_sin * cov_yaw_sin + cov_yaw_cos * cov_yaw_cos);

  geometry_msgs::msg::PoseWithCovarianceStamped estimated_odom;

  estimated_odom.pose.pose.position.x = mean_x;
  estimated_odom.pose.pose.position.y = mean_y;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, atan2(mean_yaw_sin, mean_yaw_cos));
  estimated_odom.pose.pose.orientation = tf2::toMsg(quat);

  estimated_odom.pose.covariance = {
    cov_x, 0, 0, 0, 0, 0,
    0, cov_y, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, cov_yaw};

  return estimated_odom;
}

}  // namespace nav2_localization
