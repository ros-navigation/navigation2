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
  double x_bar = prev_odom.transform.translation.x;
  double y_bar = prev_odom.transform.translation.y;
  double theta_bar = tf2::getYaw(prev_odom.transform.rotation);

  double x_bar_prime = curr_odom.transform.translation.x;
  double y_bar_prime = curr_odom.transform.translation.y;
  double theta_bar_prime = tf2::getYaw(curr_odom.transform.rotation);

  double x = prev_pose.transform.translation.x;
  double y = prev_pose.transform.translation.y;
  double theta = tf2::getYaw(prev_pose.transform.rotation);

  double delta_rot_1 = AngleUtils::angleDiff(
    atan2(y_bar_prime - y_bar, x_bar_prime - x_bar),
    theta_bar);

  if (isnan(delta_rot_1) || isinf(delta_rot_1)) {
    RCLCPP_ERROR(node_->get_logger(), "delta_rot_1 is NAN or INF");
    delta_rot_1 = 0.0;      // TODO(unassigned): consider a different value
  }

  // Avoid calculating this angle for very small transitions (e.g. on-the-spot rotation)
  if (hypot(x_bar_prime - x_bar, y_bar_prime - y_bar) < 0.01) {
    delta_rot_1 = 0.0;
  }

  double delta_trans = hypot(x_bar_prime - x_bar, y_bar_prime - y_bar);
  double delta_rot_2 = AngleUtils::angleDiff(
    AngleUtils::angleDiff(theta_bar_prime, theta_bar),
    delta_rot_1);

  // Treat forward and backward motion in the same way.
  // Without this a backward motion would be modelled as a 180 degree rotation, followed by
  // a translation, followed by another 180 degree rotation; as opposed to just one backward
  // translation
  double delta_rot_1_noise = std::min(
    fabs(AngleUtils::angleDiff(delta_rot_1, 0.0)),
    fabs(AngleUtils::angleDiff(delta_rot_1, M_PI)));
  double delta_rot_2_noise = std::min(
    fabs(AngleUtils::angleDiff(delta_rot_2, 0.0)),
    fabs(AngleUtils::angleDiff(delta_rot_2, M_PI)));

  std::random_device device;
  std::mt19937 generator(device());

  // Noise in the first rotation
  std::normal_distribution<double> delta_rot_1_noise_dist(0.0,
    sqrt(alpha1_ * pow(delta_rot_1_noise, 2) + alpha2_ * pow(delta_trans, 2)));
  double delta_rot_1_hat = AngleUtils::angleDiff(delta_rot_1, delta_rot_1_noise_dist(generator));

  // Noise in the translation
  std::normal_distribution<double> delta_trans_noise_dist(
    0.0,
    sqrt(
      alpha3_ * pow(delta_trans, 2) +
      alpha4_ * (pow(delta_rot_1_noise, 2) +
      pow(delta_rot_2_noise, 2))));
  double delta_trans_hat = delta_trans - delta_trans_noise_dist(generator);

  // Noise in the second rotation
  std::normal_distribution<double> delta_rot_2_noise_dist(
    0.0,
    sqrt(alpha1_ * pow(delta_rot_2_noise, 2) + alpha2_ * pow(delta_trans, 2)));
  double delta_rot_2_hat = AngleUtils::angleDiff(delta_rot_2, delta_rot_2_noise_dist(generator));

  geometry_msgs::msg::TransformStamped most_likely_pose;
  most_likely_pose.transform.translation.x = x + delta_trans_hat * cos(theta + delta_rot_1_hat);
  most_likely_pose.transform.translation.y = y + delta_trans_hat * sin(theta + delta_rot_1_hat);

  tf2::Quaternion theta_prime_quat;
  double theta_prime = theta + delta_rot_1_hat + delta_rot_2_hat;
  theta_prime_quat.setRPY(0.0, 0.0, theta_prime);
  most_likely_pose.transform.rotation = tf2::toMsg(theta_prime_quat);

  return most_likely_pose;
}

void DiffDriveOdomMotionModel::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  node_ = node;

  node_->declare_parameter("alpha1", 0.2);
  node_->declare_parameter("alpha2", 0.2);
  node_->declare_parameter("alpha3", 0.2);
  node_->declare_parameter("alpha4", 0.2);

  // set noise parameters
  node_->get_parameter("alpha1", alpha1_);
  node_->get_parameter("alpha2", alpha2_);
  node_->get_parameter("alpha3", alpha3_);
  node_->get_parameter("alpha4", alpha4_);
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
