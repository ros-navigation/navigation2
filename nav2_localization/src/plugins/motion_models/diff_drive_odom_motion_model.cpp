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

#include <cmath>
#include <random>
#include <memory>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/plugins/sample_motion_models/sample_motion_model_base.hpp"
#include "nav2_localization/plugins/sample_motion_models/diff_drive_odom_motion_model.hpp"
#include "angles/angles.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace nav2_localization
{

geometry_msgs::msg::Pose DiffDriveOdomMotionModel::getMostLikelyPose(
  const nav_msgs::msg::Odometry & prev_odom,
  const nav_msgs::msg::Odometry & curr_odom,
  const geometry_msgs::msg::Pose & prev_pose)
{
  MotionComponents ideal_motion_components = calculateIdealMotionComponents(prev_odom.pose.pose, curr_odom.pose.pose);

  double rot_1_hat = calculateNoisyRot(
    ideal_motion_components.rot_1_,
    ideal_motion_components.trans_);
  double trans_hat = calculateNoisyTrans(
    ideal_motion_components.rot_1_,
    ideal_motion_components.trans_,
    ideal_motion_components.rot_2_);
  double rot_2_hat = calculateNoisyRot(
    ideal_motion_components.rot_2_,
    ideal_motion_components.trans_);

  double x = prev_pose.position.x;
  double y = prev_pose.position.y;
  double theta = tf2::getYaw(prev_pose.orientation);

  geometry_msgs::msg::Pose estimated_pose;
  estimated_pose.position.x =
    x + trans_hat * cos(theta + rot_1_hat);
  estimated_pose.position.y =
    y + trans_hat * sin(theta + rot_1_hat);

  tf2::Quaternion theta_prime_quat;
  double theta_prime = theta + rot_1_hat + rot_2_hat;
  theta_prime_quat.setRPY(0.0, 0.0, theta_prime);
  estimated_pose.orientation = tf2::toMsg(theta_prime_quat);

  return estimated_pose;
}

DiffDriveOdomMotionModel::MotionComponents DiffDriveOdomMotionModel::calculateIdealMotionComponents(
  const geometry_msgs::msg::Pose & prev,
  const geometry_msgs::msg::Pose & curr)
{
  double x = prev.position.x;
  double y = prev.position.y;
  double theta = tf2::getYaw(prev.orientation);

  double x_prime = curr.position.x;
  double y_prime = curr.position.y;
  double theta_prime = tf2::getYaw(curr.orientation);

  double trans = hypot(x_prime - x, y_prime - y);

  // Assume the first rotation does not exceed +/- 90 degrees. This is done to
  // account for situtations where the robot is moving backwards. If the first
  // rotation is not restricted, a robot moving backwards in a straight line
  // will be modelled by a 180 rotation, followed by a translation then a 180
  // degree rotation in the oppsite
  double rot_1 = angles::normalize_angle(atan2(y_prime - y, x_prime - x) - theta);
  if (rot_1 < -M_PI_2) {
    rot_1 += M_PI;
    trans *= -1;    // moving backwards
  } else if (rot_1 > M_PI_2) {
    rot_1 -= M_PI;
    trans *= -1;    // moving backwards
  }

  if (isnan(rot_1) || isinf(rot_1)) {
    RCLCPP_ERROR(node_->get_logger(), "rot_1 is NAN or INF");
    rot_1 = 0.0;      // TODO(unassigned): consider a different value
  }

  // Avoid calculating this angle for very small transitions (e.g. on-the-spot rotation)
  if (hypot(x_prime - x, y_prime - y) < 0.01) {
    rot_1 = 0.0;
  }

  double rot_2 = angles::normalize_angle(angles::normalize_angle(theta_prime - theta) - rot_1);

  return MotionComponents(rot_1, trans, rot_2);
}

double DiffDriveOdomMotionModel::calculateNoisyRot(const double & rot, const double & trans)
{
  std::normal_distribution<double> rot_noise_dist(0.0,
    sqrt(
      rot_rot_noise_parm_ * pow(rot, 2) +
      trans_rot_noise_parm_ * pow(trans, 2)));
  return angles::normalize_angle(rot - rot_noise_dist(*rand_num_gen_));
}

double DiffDriveOdomMotionModel::calculateNoisyTrans(
  const double & rot1, const double & trans, const double & rot2)
{
  std::normal_distribution<double> trans_noise_dist(
    0.0,
    sqrt(
      trans_trans_noise_parm_ * pow(trans, 2) +
      rot_trans_noise_param_ * (pow(rot1, 2) +
      pow(rot2, 2))));
  return trans - trans_noise_dist(*rand_num_gen_);
}

void DiffDriveOdomMotionModel::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  node_ = node;

  node_->declare_parameter("rot_rot_noise", 0.2);
  node_->declare_parameter("trans_rot_noise", 0.2);
  node_->declare_parameter("trans_trans_noise", 0.2);
  node_->declare_parameter("rot_trans_noise", 0.2);

  // set noise parameters
  node_->get_parameter("rot_rot_noise", rot_rot_noise_parm_);
  node_->get_parameter("trans_rot_noise", trans_rot_noise_parm_);
  node_->get_parameter("trans_trans_noise", trans_trans_noise_parm_);
  node_->get_parameter("rot_trans_noise", rot_trans_noise_param_);

  std::random_device rand_device;
  rand_num_gen_ = std::make_shared<std::mt19937>(rand_device());
}

void DiffDriveOdomMotionModel::activate()
{}

void DiffDriveOdomMotionModel::deactivate()
{}

void DiffDriveOdomMotionModel::cleanup()
{}

}  // namespace nav2_localization

PLUGINLIB_EXPORT_CLASS(
  nav2_localization::DiffDriveOdomMotionModel,
  nav2_localization::SampleMotionModel)
