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

#include <memory>
#include <random>

#include "nav2_localization/plugins/sample_motion_models/diff_drive_odom_motion_model.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"

using nav2_localization::DiffDriveOdomMotionModel;

class DiffDriveOdomMotionModelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    seed_ = 1;
    rand_num_gen_ = std::make_shared<std::mt19937>(seed_);

    alpha1_ = 0.2;
    alpha2_ = 0.2;
    alpha3_ = 0.2;
    alpha4_ = 0.2;

    node_ = std::make_shared<nav2_util::LifecycleNode>(
      "sample_diff_drive_odom_motion_model_test",
      "", true);

    motion_model_.configure(node_, seed_);
    node_->set_parameter(rclcpp::Parameter("alpha1", alpha1_));
    node_->set_parameter(rclcpp::Parameter("alpha2", alpha2_));
    node_->set_parameter(rclcpp::Parameter("alpha3", alpha3_));
    node_->set_parameter(rclcpp::Parameter("alpha4", alpha4_));
    motion_model_.activate();
  }

  geometry_msgs::msg::TransformStamped createTransformStampedMsg(
    const double x, const double y,
    const double z, const double theta)
  {
    geometry_msgs::msg::TransformStamped result;
    result.transform.translation.x = x;
    result.transform.translation.y = y;
    result.transform.translation.z = z;

    tf2::Quaternion angle;
    angle.setRPY(0.0, 0.0, theta);
    result.transform.rotation = tf2::toMsg(angle);

    return result;
  }

  nav2_util::LifecycleNode::SharedPtr node_;
  DiffDriveOdomMotionModel motion_model_;
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  unsigned int seed_;
  std::shared_ptr<std::mt19937> rand_num_gen_;
};

TEST_F(DiffDriveOdomMotionModelTest, MoveForward)
{
  double x_bar = 0.0;
  double y_bar = 0.0;
  double theta_bar = 0.0;
  geometry_msgs::msg::TransformStamped prev_odom = createTransformStampedMsg(
    x_bar, y_bar, 0.0,
    theta_bar);

  double x_bar_prime = 1.0;
  double y_bar_prime = 0.0;
  double theta_bar_prime = 0.0;
  geometry_msgs::msg::TransformStamped curr_odom = createTransformStampedMsg(
    x_bar_prime,
    y_bar_prime, 0.0,
    theta_bar_prime);

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  geometry_msgs::msg::TransformStamped prev_pose = createTransformStampedMsg(x, y, 0.0, theta);

  double delta_rot_1 = atan2(y_bar_prime - y_bar, x_bar_prime - x_bar) - theta_bar_prime;
  double delta_trans = hypot(x_bar_prime - x_bar, y_bar_prime - y_bar);
  double delta_rot_2 = theta_bar_prime - theta_bar - delta_rot_1;

  double rot_1_std = sqrt(alpha1_ * pow(delta_rot_1, 2) + alpha2_ * pow(delta_trans, 2));
  double trans_std = sqrt(
    alpha3_ * pow(delta_trans, 2) +
    alpha4_ * pow(delta_rot_1, 2) +
    alpha4_ * pow(delta_rot_2, 2));
  double rot_2_std = sqrt(alpha1_ * pow(delta_rot_2, 2) + alpha2_ * pow(delta_trans, 2));

  std::normal_distribution<double> rot_1_dist(0.0, rot_1_std);
  std::normal_distribution<double> trans_dist(0.0, trans_std);
  std::normal_distribution<double> rot_2_dist(0.0, rot_2_std);

  double delta_rot_1_hat = delta_rot_1 - rot_1_dist(*rand_num_gen_);
  double delta_trans_hat = delta_trans - trans_dist(*rand_num_gen_);
  double delta_rot_2_hat = delta_rot_2 - rot_2_dist(*rand_num_gen_);

  double x_prime = x + delta_trans_hat * cos(theta + delta_rot_1_hat);
  double y_prime = y + delta_trans_hat * sin(theta + delta_rot_1_hat);
  double theta_prime = theta + delta_rot_1_hat + delta_rot_2_hat;

  geometry_msgs::msg::TransformStamped expected =
    createTransformStampedMsg(x_prime, y_prime, 0.0, theta_prime);
  geometry_msgs::msg::TransformStamped actual =
    motion_model_.getMostLikelyPose(prev_odom, curr_odom, prev_pose);

  ASSERT_EQ(expected, actual);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return all_successful;
}
