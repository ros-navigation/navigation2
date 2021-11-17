// Copyright (c) 2021 Khaled SAAD, Jose M. TORRES-CAMARA and Marwan TAHER
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
#include "tf2/utils.h"

using nav2_localization::DiffDriveOdomMotionModel;

class DiffDriveTestFixture : public DiffDriveOdomMotionModel, public ::testing::Test
{
protected:
  void SetUp() override
  {
    epsilon = 1e-4;
  }

  geometry_msgs::msg::Pose createPoseMsg(
    const double x, const double y, const double theta)
  {
    geometry_msgs::msg::Pose result;
    result.position.x = x;
    result.position.y = y;
    result.position.z = 0.0;

    tf2::Quaternion angle;
    angle.setRPY(0.0, 0.0, theta);
    result.orientation = tf2::toMsg(angle);

    return result;
  }

  nav_msgs::msg::Odometry createOdomMsg(
    const double x, const double y, const double theta)
  {
    nav_msgs::msg::Odometry result;
    result.pose.pose.position.x = x;
    result.pose.pose.position.y = y;
    result.pose.pose.position.z = 0.0;

    tf2::Quaternion angle;
    angle.setRPY(0.0, 0.0, theta);
    result.pose.pose.orientation = tf2::toMsg(angle);

    return result;
  }

  double epsilon;
};

TEST_F(DiffDriveTestFixture, IdealMotionTest)
{
  // no motion
  geometry_msgs::msg::Pose prev = createPoseMsg(2.0, 2.0, 0.0);
  geometry_msgs::msg::Pose curr = createPoseMsg(2.0, 2.0, 0.0);
  MotionComponents motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, 0.0, epsilon);
  EXPECT_NEAR(motion_components.trans_, 0.0, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, 0.0, epsilon);

  // forward motion
  prev = createPoseMsg(0.0, 2.0, 0.0);
  curr = createPoseMsg(2.0, 2.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, 0.0, epsilon);
  EXPECT_NEAR(motion_components.trans_, 2.0, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, 0.0, epsilon);

  // backward motion
  prev = createPoseMsg(2.0, 2.0, 0.0);
  curr = createPoseMsg(0.0, 2.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, 0.0, epsilon);
  EXPECT_NEAR(motion_components.trans_, -2.0, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, 0.0, epsilon);

  // side motion
  prev = createPoseMsg(0.0, 0.0, 0.0);
  curr = createPoseMsg(0.0, -2.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, -M_PI_2, epsilon);
  EXPECT_NEAR(motion_components.trans_, 2.0, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, M_PI_2, epsilon);

  prev = createPoseMsg(0.0, 0.0, 0.0);
  curr = createPoseMsg(0.0, 2.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, M_PI_2, epsilon);
  EXPECT_NEAR(motion_components.trans_, 2.0, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, -M_PI_2, epsilon);

  // on-the-spot rotation
  // counter clockwise
  prev = createPoseMsg(2.0, 2.0, 0.0);
  curr = createPoseMsg(2.0, 2.0, M_PI_2);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, 0.0, epsilon);
  EXPECT_NEAR(motion_components.trans_, 0.0, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, M_PI_2, epsilon);

  // clockwise
  prev = createPoseMsg(2.0, 2.0, 0.0);
  curr = createPoseMsg(2.0, 2.0, -M_PI_2);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, 0.0, epsilon);
  EXPECT_NEAR(motion_components.trans_, 0.0, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, -M_PI_2, epsilon);

  // diagonal motion
  // top left
  prev = createPoseMsg(0.0, 0.0, 0.0);
  curr = createPoseMsg(2.0, 2.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, M_PI_4, epsilon);
  EXPECT_NEAR(motion_components.trans_, 2.8284, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, -M_PI_4, epsilon);

  // bottom left
  prev = createPoseMsg(0.0, 0.0, 0.0);
  curr = createPoseMsg(-2.0, 2.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, -M_PI_4, epsilon);
  EXPECT_NEAR(motion_components.trans_, -2.8284, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, M_PI_4, epsilon);

  // top right
  prev = createPoseMsg(0.0, 0.0, 0.0);
  curr = createPoseMsg(2.0, -2.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, -M_PI_4, epsilon);
  EXPECT_NEAR(motion_components.trans_, 2.8284, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, M_PI_4, epsilon);

  // bottom right
  prev = createPoseMsg(0.0, 0.0, 0.0);
  curr = createPoseMsg(-2.0, -2.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  EXPECT_NEAR(motion_components.rot_1_, M_PI_4, epsilon);
  EXPECT_NEAR(motion_components.trans_, -2.8284, epsilon);
  EXPECT_NEAR(motion_components.rot_2_, -M_PI_4, epsilon);
}

TEST_F(DiffDriveTestFixture, NoisyRotTest)
{
  // no noise
  rot_rot_noise_parm_ = 0.0;
  trans_rot_noise_parm_ = 0.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);

  double rot = M_PI_4;
  double trans = 2.0;
  double noisy_rot = calculateNoisyRot(rot, trans);
  EXPECT_NEAR(rot, noisy_rot, epsilon);

  // rot/rot noise only
  rot_rot_noise_parm_ = 0.2;
  trans_rot_noise_parm_ = 0.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);

  rot = M_PI_4;
  trans = 2.0;
  noisy_rot = calculateNoisyRot(rot, trans);
  // distribution variance = rot_rot_noise_parm_*rot^2 = 0.2*(pi/4)^2 = 0.1234
  // output of the distribution given the rand number generator seeded with 0: 0.3944
  EXPECT_NEAR(noisy_rot, rot - 0.3944, epsilon);
  // changing trans should have no impact
  trans = 5.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_rot = calculateNoisyRot(rot, trans);
  EXPECT_NEAR(noisy_rot, rot - 0.3944, epsilon);

  // rot/trans noise only
  rot_rot_noise_parm_ = 0.0;
  trans_rot_noise_parm_ = 0.2;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);

  rot = M_PI_4;
  trans = 2.0;
  noisy_rot = calculateNoisyRot(rot, trans);
  // distribution variance = trans_rot_noise_parm_*trans^2 = 0.2*2.0^2 = 0.8
  // output of the distribution given the rand number generator seeded with 0: 1.0043
  EXPECT_NEAR(noisy_rot, rot - 1.0043, epsilon);
  // changing rot should have no impact
  rot = 0.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_rot = calculateNoisyRot(rot, trans);
  EXPECT_NEAR(noisy_rot, rot - 1.0043, epsilon);

  // both noise parameters set
  rot_rot_noise_parm_ = 0.2;
  trans_rot_noise_parm_ = 0.3;

  // rot only
  rot = M_PI_4;
  trans = 0.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_rot = calculateNoisyRot(rot, trans);
  // distribution variance = rot_rot_noise_parm_*rot^2 = 0.2*(pi/4)^2 = 0.1234
  // output of the distribution given the rand number generator seeded with 0: 0.3944
  EXPECT_NEAR(noisy_rot, rot - 0.3944, epsilon);

  // trans only
  rot = 0.0;
  trans = 2.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_rot = calculateNoisyRot(rot, trans);
  // distribution variance = trans_rot_noise_parm_*trans^2 = 0.3*(2.0)^2 = 1.2
  // output of the distribution given the rand number generator seeded with 0: 1.2230
  EXPECT_NEAR(noisy_rot, rot - 1.2300, epsilon);

  // no trans or rot
  rot = 0.0;
  trans = 0.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_rot = calculateNoisyRot(rot, trans);
  EXPECT_NEAR(noisy_rot, 0.0, epsilon);

  // +ve rot with trans
  rot = M_PI_4;
  trans = 2.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_rot = calculateNoisyRot(rot, trans);
  // distribution variance = rot_rot_noise_parm_*rot^2 + trans_rot_noise_parm_*trans^2
  //                       = 0.2*(pi/4)^2 + 0.3*2.0^2 = 1.3234
  // output of the distribution given the rand number generator seeded with 0: 1.2917
  EXPECT_NEAR(noisy_rot, rot - 1.2917, epsilon);

  // -ve rot with trans
  rot = -M_PI_4;
  trans = 2.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_rot = calculateNoisyRot(rot, trans);
  // distribution variance = rot_rot_noise_parm_*rot^2 + trans_rot_noise_parm_*trans^2
  //                       = 0.2*(pi/4)^2 + 0.3*2.0^2 = 1.3234
  // output of the distribution given the rand number generator seeded with 0: 1.2917
  EXPECT_NEAR(noisy_rot, rot - 1.2917, epsilon);
}

TEST_F(DiffDriveTestFixture, NoisyTransTest)
{
  // no noise
  trans_trans_noise_parm_ = 0.0;
  rot_trans_noise_param_ = 0.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);

  double rot1 = M_PI_4;
  double trans = 2.0;
  double rot2 = M_PI_4;
  double noisy_trans = calculateNoisyTrans(rot1, trans, rot2);
  ASSERT_NEAR(noisy_trans, trans, epsilon);

  // trans/trans noise only
  trans_trans_noise_parm_ = 0.2;
  rot_trans_noise_param_ = 0.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);

  rot1 = M_PI_4;
  trans = 2.0;
  rot2 = M_PI_4;
  noisy_trans = calculateNoisyTrans(rot1, trans, rot2);
  // distribution variance = trans_trans_noise_parm_*trans^2 = 0.2*(2.0)^2 = 0.8
  // output of the distribution given the rand number generator seeded with 0: 1.0043
  ASSERT_NEAR(noisy_trans, trans - 1.0043, epsilon);
  // changing rot1 and 2 should have no impact
  rot1 = M_PI;
  rot2 = -M_PI_2;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_trans = calculateNoisyTrans(rot1, trans, rot2);
  ASSERT_NEAR(noisy_trans, trans - 1.0043, epsilon);

  // rot/trans noise only
  trans_trans_noise_parm_ = 0.0;
  rot_trans_noise_param_ = 0.2;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);

  rot1 = M_PI_4;
  trans = 2.0;
  rot2 = M_PI_4;
  noisy_trans = calculateNoisyTrans(rot1, trans, rot2);
  // distribution variance = rot_trans_noise_parm_*(rot1^2 + rot2^2) = 0.2*(2*(pi/4)^2) = 0.2467
  // output of the distribution given the rand number generator seeded with 0: 0.5577
  ASSERT_NEAR(noisy_trans, trans - 0.5577, epsilon);
  // changing trans should have no impact
  trans = 5.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_trans = calculateNoisyTrans(rot1, trans, rot2);
  ASSERT_NEAR(noisy_trans, trans - 0.5577, epsilon);

  // both noise parameters set
  trans_trans_noise_parm_ = 0.2;
  rot_trans_noise_param_ = 0.3;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);

  // straight line motion - forward
  rot1 = 0.0;
  trans = 2.0;
  rot2 = 0.0;
  noisy_trans = calculateNoisyTrans(rot1, trans, rot2);
  // distribution variance = trans_trans_noise_parm_*trans^2 = 0.2*(2.0)^2 = 0.8
  // output of the distribution given the rand number generator seeded with 0: 1.0043
  ASSERT_NEAR(noisy_trans, trans - 1.0043, epsilon);

  // straight line motion - backward
  rot1 = 0.0;
  trans = -2.0;
  rot2 = 0.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_trans = calculateNoisyTrans(rot1, trans, rot2);
  // distribution variance = trans_trans_noise_parm_*trans^2 = 0.2*(2.0)^2 = 0.8
  // output of the distribution given the rand number generator seeded with 0: 1.0043
  ASSERT_NEAR(noisy_trans, trans - 1.0043, epsilon);

  // on-the-spot rotation
  rot1 = 0.0;
  trans = 0.0;
  rot2 = M_PI_2;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_trans = calculateNoisyTrans(rot1, trans, rot2);
  // distribution variance = rot_trans_noise_parm_*rot2^2 = 0.3*(pi/2)^2 = 0.7402
  // output of the distribution given the rand number generator seeded with 0: 0.9660
  ASSERT_NEAR(noisy_trans, -0.9660, epsilon);

  // both angles set
  rot1 = M_PI_4;
  trans = 2.0;
  rot2 = M_PI_2;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  noisy_trans = calculateNoisyTrans(rot1, trans, rot2);
  // distribution variance = trans_trans_noise_parm_*trans^2 +
  //                         rot_trans_noise_parm_*(rot1^2 + rot2^2)
  //                       = 0.2*(2.0)^2 + 0.3*((pi/4)^2 + (pi/2)^2) = 1.7253
  // output of the distribution given the rand number generator seeded with 0: 1.4748
  ASSERT_NEAR(noisy_trans, trans - 1.4748, epsilon);
}

TEST_F(DiffDriveTestFixture, MostLikelyPoseTest)
{
  rand_num_gen_ = std::make_shared<std::mt19937>(0);

  // no noise
  rot_rot_noise_parm_ = 0.0;
  trans_rot_noise_parm_ = 0.0;
  trans_trans_noise_parm_ = 0.0;
  rot_trans_noise_param_ = 0.0;

  // forward motion
  auto prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  auto curr_odom = createOdomMsg(2.0, 0.0, 0.0);
  auto prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  auto expected_pose = createPoseMsg(2.0, 0.0, 0.0);
  auto actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  double expected_angle = 0.0;
  double actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);

  // backward motion
  prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  curr_odom = createOdomMsg(-2.0, 0.0, 0.0);
  prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  expected_pose = createPoseMsg(-2.0, 0.0, 0.0);
  actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  expected_angle = 0.0;
  actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);

  // side motion - right
  prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  curr_odom = createOdomMsg(0.0, -2.0, 0.0);
  prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  expected_pose = createPoseMsg(0.0, -2.0, 0.0);
  actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  expected_angle = 0.0;
  actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);

  // side motion - left
  prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  curr_odom = createOdomMsg(0.0, 2.0, 0.0);
  prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  expected_pose = createPoseMsg(0.0, 2.0, 0.0);
  actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  expected_angle = 0.0;
  actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);

  // CCW rotation
  prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  curr_odom = createOdomMsg(0.0, 0.0, M_PI_4);
  prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  expected_pose = createPoseMsg(0.0, 0.0, M_PI_4);
  actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  expected_angle = M_PI_4;
  actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);

  // CW rotation
  prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  curr_odom = createOdomMsg(0.0, 0.0, -M_PI_4);
  prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  expected_pose = createPoseMsg(0.0, 0.0, -M_PI_4);
  actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  expected_angle = -M_PI_4;
  actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);

  // diagonal motion - top left
  prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  curr_odom = createOdomMsg(2.0, 2.0, 0.0);
  prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  expected_pose = createPoseMsg(2.0, 2.0, 0.0);
  actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  expected_angle = 0.0;
  actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);

  // diagonal motion - bottom left
  prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  curr_odom = createOdomMsg(-2.0, 2.0, 0.0);
  prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  expected_pose = createPoseMsg(-2.0, 2.0, 0.0);
  actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  expected_angle = 0.0;
  actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);

  // diagonal motion - top right
  prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  curr_odom = createOdomMsg(2.0, -2.0, 0.0);
  prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  expected_pose = createPoseMsg(2.0, -2.0, 0.0);
  actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  expected_angle = 0.0;
  actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);

  // diagonal motion - bottom right
  prev_odom = createOdomMsg(0.0, 0.0, 0.0);
  curr_odom = createOdomMsg(-2.0, -2.0, 0.0);
  prev_pose = createPoseMsg(0.0, 0.0, 0.0);

  expected_pose = createPoseMsg(-2.0, -2.0, 0.0);
  actual_pose = getMostLikelyPose(prev_odom, curr_odom, prev_pose);
  expected_angle = 0.0;
  actual_angle = tf2::getYaw(actual_pose.orientation);

  EXPECT_NEAR(expected_pose.position.x, actual_pose.position.x, epsilon);
  EXPECT_NEAR(expected_pose.position.y, actual_pose.position.y, epsilon);
  EXPECT_NEAR(expected_angle, actual_angle, epsilon);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return all_successful;
}
