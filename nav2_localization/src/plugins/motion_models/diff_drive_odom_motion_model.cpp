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
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/plugins/sample_motion_models/diff_drive_odom_motion_model.hpp"
#include "nav2_localization/angle_utils.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace nav2_localization
{

geometry_msgs::msg::TransformStamped DiffDriveOdomMotionModel::getMostLikelyPose(
  const geometry_msgs::msg::TransformStamped & prev_odom,
  const geometry_msgs::msg::TransformStamped & curr_odom,
  const geometry_msgs::msg::TransformStamped & prev_pose)
{
  MotionComponents ideal_motion = calculateIdealMotionComponents(prev_odom, curr_odom);
  MotionComponents noisy_motion = calculateNoisyMotionComponents(ideal_motion);
  return estimateCurrentPose(prev_pose, noisy_motion);
}

DiffDriveOdomMotionModel::MotionComponents DiffDriveOdomMotionModel::calculateIdealMotionComponents(
  const geometry_msgs::msg::TransformStamped & prev,
  const geometry_msgs::msg::TransformStamped & curr)
{
  double x = prev.transform.translation.x;
  double y = prev.transform.translation.y;
  double theta = tf2::getYaw(prev.transform.rotation);

  double x_prime = curr.transform.translation.x;
  double y_prime = curr.transform.translation.y;
  double theta_prime = tf2::getYaw(curr.transform.rotation);

  double rot_1 = AngleUtils::angleDiff(atan2(y_prime - y, x_prime - x), theta);

  if (isnan(rot_1) || isinf(rot_1)) {
    RCLCPP_ERROR(node_->get_logger(), "rot_1 is NAN or INF");
    rot_1 = 0.0;      // TODO(unassigned): consider a different value
  }

  // Avoid calculating this angle for very small transitions (e.g. on-the-spot rotation)
  if (hypot(x_prime - x, y_prime - y) < 0.01) {
    rot_1 = 0.0;
  }

  double trans = hypot(x_prime - x, y_prime - y);
  double rot_2 = AngleUtils::angleDiff(AngleUtils::angleDiff(theta_prime, theta), rot_1);

  return MotionComponents(rot_1, trans, rot_2);
}

DiffDriveOdomMotionModel::MotionComponents DiffDriveOdomMotionModel::calculateNoisyMotionComponents(
  const MotionComponents & ideal)
{
  // Treat forward and backward motion in the same way.
  // Without this a backward motion would be modelled as a 180 degree rotation, followed by
  // a translation, followed by another 180 degree rotation; as opposed to just one backward
  // translation
  double ideal_rot_1_normal = std::min(
    fabs(AngleUtils::angleDiff(ideal.rot_1_, 0.0)),
    fabs(AngleUtils::angleDiff(ideal.rot_1_, M_PI)));
  double ideal_rot_2_normal = std::min(
    fabs(AngleUtils::angleDiff(ideal.rot_2_, 0.0)),
    fabs(AngleUtils::angleDiff(ideal.rot_2_, M_PI)));

  // Noise in the first rotation
  std::normal_distribution<double> rot_1_noise_dist(0.0,
    sqrt(
      rot_rot_noise_parm_ * pow(ideal_rot_1_normal, 2) +
      trans_rot_noise_parm_ * pow(ideal.trans_, 2)));
  double rot_1_hat = AngleUtils::angleDiff(ideal.rot_1_, rot_1_noise_dist(*rand_num_gen_));

  // Noise in the translation
  std::normal_distribution<double> trans_noise_dist(
    0.0,
    sqrt(
      trans_trans_noise_parm_ * pow(ideal.trans_, 2) +
      rot_trans_noise_param_ * (pow(ideal_rot_1_normal, 2) +
      pow(ideal_rot_2_normal, 2))));
  double trans_hat = ideal.trans_ - trans_noise_dist(*rand_num_gen_);

  // Noise in the second rotation
  std::normal_distribution<double> rot_2_noise_dist(
    0.0,
    sqrt(
      rot_rot_noise_parm_ * pow(ideal_rot_2_normal, 2) +
      trans_rot_noise_parm_ * pow(ideal.trans_, 2)));
  double rot_2_hat = AngleUtils::angleDiff(ideal.rot_2_, rot_2_noise_dist(*rand_num_gen_));

  return MotionComponents(rot_1_hat, trans_hat, rot_2_hat);
}

geometry_msgs::msg::TransformStamped DiffDriveOdomMotionModel::estimateCurrentPose(
  const geometry_msgs::msg::TransformStamped & prev_pose,
  const MotionComponents & noisy_motion_components
)
{
  double x = prev_pose.transform.translation.x;
  double y = prev_pose.transform.translation.y;
  double theta = tf2::getYaw(prev_pose.transform.rotation);

  geometry_msgs::msg::TransformStamped estimated_pose;
  estimated_pose.transform.translation.x =
    x + noisy_motion_components.trans_ * cos(theta + noisy_motion_components.rot_1_);
  estimated_pose.transform.translation.y =
    y + noisy_motion_components.trans_ * sin(theta + noisy_motion_components.rot_1_);

  tf2::Quaternion theta_prime_quat;
  double theta_prime = theta + noisy_motion_components.rot_1_ + noisy_motion_components.rot_2_;
  theta_prime_quat.setRPY(0.0, 0.0, theta_prime);
  estimated_pose.transform.rotation = tf2::toMsg(theta_prime_quat);

  return estimated_pose;
}

void DiffDriveOdomMotionModel::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const unsigned int & rand_num_gen_seed)
{
  node_ = node;

  // set noise parameters
  node_->get_parameter("rot_rot_noise", rot_rot_noise_parm_);
  node_->get_parameter("trans_rot_noise", trans_rot_noise_parm_);
  node_->get_parameter("trans_trans_noise", trans_trans_noise_parm_);
  node_->get_parameter("rot_trans_noise", rot_trans_noise_param_);

  node_->declare_parameter("rot_rot_noise", 0.2);
  node_->declare_parameter("trans_rot_noise", 0.2);
  node_->declare_parameter("trans_trans_noise", 0.2);
  node_->declare_parameter("rot_trans_noise", 0.2);

  rand_num_gen_ = std::make_shared<std::mt19937>(rand_num_gen_seed);
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
