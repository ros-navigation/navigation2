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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return all_successful;
}
