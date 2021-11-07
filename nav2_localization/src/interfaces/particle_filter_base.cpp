#define _USE_MATH_DEFINES
#include <math.h>

#include <random>

#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

#include "nav2_localization/interfaces/particle_filter_base.hpp"

namespace nav2_localization
{
ParticleFilter::ParticleFilter() {}

void ParticleFilter::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  SampleMotionModel::Ptr & motionSampler,
  Matcher2d::Ptr & matcher)
{
  Solver::configure(node, motionSampler, matcher);

  node_->declare_parameter("particles_count", 500);
  node_->declare_parameter("initialization_radius", 1.0);
  node_->declare_parameter("particles_spread_yaw", 1.57);

  node_->get_parameter("particles_count_", particles_count_);
  node_->get_parameter("particles_spread_radius", particles_spread_radius_);
  node_->get_parameter("particles_spread_yaw", particles_spread_yaw_);
}

void ParticleFilter::initFilter(const geometry_msgs::msg::Pose & init_pose, const nav_msgs::msg::Odometry & init_odom)
{
  Solver::initFilter(init_pose, init_odom);

  // Initialize particles randomly around init_pose within a radius particles_spread_radius_
  geometry_msgs::msg::Pose temp_pose;

  for (int i = 0; i < particles_count_; i++) {
    std::random_device rand_device;
    std::mt19937 gen(rand_device());
    std::uniform_real_distribution<> pose_dist;
    std::uniform_real_distribution<> yaw_dist(-particles_spread_yaw_, particles_spread_yaw_);

    // Get (x, y) coordinates that fall inside a circle of radius particles_spread_radius_
    do {
      temp_pose.position.x = init_pose.position.x + particles_spread_radius_ * pose_dist(gen);
      temp_pose.position.y = init_pose.position.y + particles_spread_radius_ * pose_dist(gen);
    } while (pow(temp_pose.position.x, 2) + pow(temp_pose.position.y, 2)
            < pow(particles_spread_radius_, 2));

    double yaw = tf2::getYaw(init_pose.orientation) + yaw_dist(gen);

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    temp_pose.orientation = tf2::toMsg(quat);

    particles_.push_back(Particle(temp_pose, 1));
  }
}

void ParticleFilter::resample()
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

geometry_msgs::msg::PoseWithCovarianceStamped ParticleFilter::getMeanPose()
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

  geometry_msgs::msg::PoseWithCovarianceStamped estimated_odom;

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
