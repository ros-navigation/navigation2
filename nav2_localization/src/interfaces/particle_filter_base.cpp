#define _USE_MATH_DEFINES
#include <math.h>

#include <random>

#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

#include "nav2_msgs/msg/particle.hpp"

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

  particle_cloud_pub_ = node_->create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud", rclcpp::SensorDataQoS());

  particles_poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
    "particlecloud", rclcpp::SensorDataQoS());

  node_->declare_parameter("particles_count", 500);
  node_->declare_parameter("initialization_radius", 1.0);
  node_->declare_parameter("particles_spread_yaw", 1.57);

  node_->get_parameter("particles_count", particles_count_);
  node_->get_parameter("particles_spread_radius", particles_spread_radius_);
  node_->get_parameter("particles_spread_yaw", particles_spread_yaw_);
}

void ParticleFilter::activate()
{
  particle_cloud_pub_->on_activate();
  particles_poses_pub_->on_activate();
}

void ParticleFilter::initPose(const geometry_msgs::msg::PoseWithCovarianceStamped & init_pose)
{
  Solver::initPose(init_pose);

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

  visualize_particles();
}

void ParticleFilter::resample()
{
  std::vector<Particle> sampled_particles;

  double particle_count_dtype = static_cast<double>(particles_count_);

  std::random_device rand_device;
  std::mt19937 gen(rand_device());
  std::uniform_real_distribution<double> dist(0, 1 / particle_count_dtype);

  double c = particles_[0].weight_;
  double r = dist(gen);

  int i = 0;
  for (int m = 0; m < particles_count_; m++) {
    double u = r + (m / particle_count_dtype);
    while (u > c) {
      i++;
      c += particles_[i].weight_;
    }
    sampled_particles.push_back(particles_[i]);
  }

  particles_ = sampled_particles;
}

void ParticleFilter::visualize_particles()
{
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
