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

class DiffDriveTestFixture : public DiffDriveOdomMotionModel, public ::testing::Test
{
protected:
  void SetUp() override
  {
    epsilon = 1e-3;
  }

  geometry_msgs::msg::TransformStamped createTransformStampedMsg(
    const double x, const double y, const double z, const double theta)
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

  double epsilon;
};

TEST_F(DiffDriveTestFixture, IdealMotionTest)
{
  // no motion
  geometry_msgs::msg::TransformStamped prev = createTransformStampedMsg(2.0, 2.0, 0.0, 0.0);
  geometry_msgs::msg::TransformStamped curr = createTransformStampedMsg(2.0, 2.0, 0.0, 0.0);
  MotionComponents motion_components = calculateIdealMotionComponents(prev, curr);

  ASSERT_NEAR(motion_components.rot_1_, 0.0, epsilon);
  ASSERT_NEAR(motion_components.trans_, 0.0, epsilon);
  ASSERT_NEAR(motion_components.rot_2_, 0.0, epsilon);

  // forward motion
  prev = createTransformStampedMsg(0.0, 2.0, 0.0, 0.0);
  curr = createTransformStampedMsg(2.0, 2.0, 0.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  ASSERT_NEAR(motion_components.rot_1_, 0.0, epsilon);
  ASSERT_NEAR(motion_components.trans_, 2.0, epsilon);
  ASSERT_NEAR(motion_components.rot_2_, 0.0, epsilon);

  // backward motion
  prev = createTransformStampedMsg(2.0, 2.0, 0.0, 0.0);
  curr = createTransformStampedMsg(0.0, 2.0, 0.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  ASSERT_NEAR(motion_components.rot_1_, M_PI, epsilon);
  ASSERT_NEAR(motion_components.trans_, 2.0, epsilon);
  ASSERT_NEAR(motion_components.rot_2_, -M_PI, epsilon);

  // on-spot rotation
  // counter clockwise
  prev = createTransformStampedMsg(2.0, 2.0, 0.0, 0.0);
  curr = createTransformStampedMsg(2.0, 2.0, 0.0, M_PI_2);
  motion_components = calculateIdealMotionComponents(prev, curr);

  ASSERT_NEAR(motion_components.rot_1_, 0.0, epsilon);
  ASSERT_NEAR(motion_components.trans_, 0.0, epsilon);
  ASSERT_NEAR(motion_components.rot_2_, M_PI_2, epsilon);

  // clockwise
  prev = createTransformStampedMsg(2.0, 2.0, 0.0, 0.0);
  curr = createTransformStampedMsg(2.0, 2.0, 0.0, -M_PI_2);
  motion_components = calculateIdealMotionComponents(prev, curr);

  ASSERT_NEAR(motion_components.rot_1_, 0.0, epsilon);
  ASSERT_NEAR(motion_components.trans_, 0.0, epsilon);
  ASSERT_NEAR(motion_components.rot_2_, -M_PI_2, epsilon);

  // diagonal motion
  // top left
  prev = createTransformStampedMsg(0.0, 0.0, 0.0, 0.0);
  curr = createTransformStampedMsg(2.0, 2.0, 0.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  ASSERT_NEAR(motion_components.rot_1_, M_PI_4, epsilon);
  ASSERT_NEAR(motion_components.trans_, 2.828, epsilon);
  ASSERT_NEAR(motion_components.rot_2_, -M_PI_4, epsilon);

  // bottom left
  prev = createTransformStampedMsg(0.0, 0.0, 0.0, 0.0);
  curr = createTransformStampedMsg(-2.0, 2.0, 0.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  ASSERT_NEAR(motion_components.rot_1_, 3 * M_PI_4, epsilon);
  ASSERT_NEAR(motion_components.trans_, 2.828, epsilon);
  ASSERT_NEAR(motion_components.rot_2_, -3 * M_PI_4, epsilon);

  // top right
  prev = createTransformStampedMsg(0.0, 0.0, 0.0, 0.0);
  curr = createTransformStampedMsg(2.0, -2.0, 0.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  ASSERT_NEAR(motion_components.rot_1_, -M_PI_4, epsilon);
  ASSERT_NEAR(motion_components.trans_, 2.828, epsilon);
  ASSERT_NEAR(motion_components.rot_2_, M_PI_4, epsilon);

  // bottom right
  prev = createTransformStampedMsg(0.0, 0.0, 0.0, 0.0);
  curr = createTransformStampedMsg(-2.0, -2.0, 0.0, 0.0);
  motion_components = calculateIdealMotionComponents(prev, curr);

  ASSERT_NEAR(motion_components.rot_1_, -3 * M_PI_4, epsilon);
  ASSERT_NEAR(motion_components.trans_, 2.828, epsilon);
  ASSERT_NEAR(motion_components.rot_2_, 3 * M_PI_4, epsilon);
}

TEST_F(DiffDriveTestFixture, NoisyMotionTest)
{
  // No noise
  rot_rot_noise_parm_ = 0.0;
  trans_rot_noise_parm_ = 0.0;
  trans_trans_noise_parm_ = 0.0;
  rot_trans_noise_param_ = 0.0;
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  MotionComponents ideal_motion_components(M_PI_4, 2.0, -M_PI_4);
  MotionComponents noisy_motion_components =
    calculateNoisyMotionComponents(ideal_motion_components);

  ASSERT_NEAR(noisy_motion_components.rot_1_, M_PI_4, epsilon);
  ASSERT_NEAR(noisy_motion_components.trans_, 2.0, epsilon);
  ASSERT_NEAR(noisy_motion_components.rot_2_, -M_PI_4, epsilon);

  // rot/rot noise only
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  rot_rot_noise_parm_ = 0.1;
  trans_rot_noise_parm_ = 0.0;
  trans_trans_noise_parm_ = 0.0;
  rot_trans_noise_param_ = 0.0;
  noisy_motion_components = calculateNoisyMotionComponents(ideal_motion_components);

  ASSERT_NEAR(noisy_motion_components.rot_1_, 0.5065, epsilon);
  ASSERT_NEAR(noisy_motion_components.trans_, ideal_motion_components.trans_, epsilon);
  ASSERT_NEAR(noisy_motion_components.rot_2_, -0.4321, epsilon);

  // trans/rot only
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  rot_rot_noise_parm_ = 0.0;
  trans_rot_noise_parm_ = 0.1;
  trans_trans_noise_parm_ = 0.0;
  rot_trans_noise_param_ = 0.0;
  noisy_motion_components = calculateNoisyMotionComponents(ideal_motion_components);

  ASSERT_NEAR(noisy_motion_components.rot_1_, 0.0753, epsilon);
  ASSERT_NEAR(noisy_motion_components.trans_, ideal_motion_components.trans_, epsilon);
  ASSERT_NEAR(noisy_motion_components.rot_2_, 0.1142, epsilon);

  // trans/trans only
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  rot_rot_noise_parm_ = 0.0;
  trans_rot_noise_parm_ = 0.0;
  trans_trans_noise_parm_ = 0.1;
  rot_trans_noise_param_ = 0.0;
  noisy_motion_components = calculateNoisyMotionComponents(ideal_motion_components);

  ASSERT_NEAR(noisy_motion_components.rot_1_, ideal_motion_components.rot_1_, epsilon);
  ASSERT_NEAR(noisy_motion_components.trans_, 1.9552, epsilon);
  ASSERT_NEAR(noisy_motion_components.rot_2_, ideal_motion_components.rot_2_, epsilon);

  // rot/trans only
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  rot_rot_noise_parm_ = 0.0;
  trans_rot_noise_parm_ = 0.0;
  trans_trans_noise_parm_ = 0.0;
  rot_trans_noise_param_ = 0.1;
  noisy_motion_components = calculateNoisyMotionComponents(ideal_motion_components);

  ASSERT_NEAR(noisy_motion_components.rot_1_, ideal_motion_components.rot_1_, epsilon);
  ASSERT_NEAR(noisy_motion_components.trans_, 1.9751113, epsilon);
  ASSERT_NEAR(noisy_motion_components.rot_2_, ideal_motion_components.rot_2_, epsilon);

  // All noise parameters set
  rand_num_gen_ = std::make_shared<std::mt19937>(0);
  rot_rot_noise_parm_ = 0.1;
  trans_rot_noise_parm_ = 0.2;
  trans_trans_noise_parm_ = 0.3;
  rot_trans_noise_param_ = 0.4;
  noisy_motion_components = calculateNoisyMotionComponents(ideal_motion_components);

  ASSERT_NEAR(noisy_motion_components.rot_1_, -0.2569, epsilon);
  ASSERT_NEAR(noisy_motion_components.trans_, 1.9078, epsilon);
  ASSERT_NEAR(noisy_motion_components.rot_2_, 0.5349, epsilon);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return all_successful;
}
