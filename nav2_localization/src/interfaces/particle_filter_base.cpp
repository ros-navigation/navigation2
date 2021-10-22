#define _USE_MATH_DEFINES
#include <math.h>

#include <random>

#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

#include "nav2_localization/interfaces/particle_filter_base.hpp"

namespace nav2_localization
{
void ParticleFilter::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  SampleMotionModel::Ptr & motionSampler,
  Matcher2d::Ptr & matcher)
{
  node_ = node;
  motion_sampler_ = motionSampler;
  matcher_ = matcher;

  node_->declare_parameter("particles_count", 500);
  node_->declare_parameter("initialization_radius", 1.0);
  node_->declare_parameter("particles_spread_yaw", 1.57);

  node_->get_parameter("particles_count_", particles_count_);
  node_->get_parameter("particles_spread_radius", particles_spread_radius_);
  node_->get_parameter("particles_spread_yaw", particles_spread_yaw_);
}

void ParticleFilter::initFilter(const geometry_msgs::msg::Pose & init_pose)
{
  Solver::initFilter(init_pose);

  // Initialize particles randomly around init_pose within a radius particles_spread_radius_
  particles_.resize(particles_count_);

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
    } while (pow(temp_pose.position.x, 2) + pow(temp_pose.position.y, 2) <  pow(particles_spread_radius_, 2));

    double yaw = tf2::getYaw(init_pose.orientation) + yaw_dist(gen);

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    temp_pose.orientation = tf2::toMsg(quat);

    particles_.push_back(Particle(temp_pose, 1));
  }
}

}  // namespace nav2_localization
