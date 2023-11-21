// Copyright (c) 2023 Alberto J. Tudela Roldán
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
// limitations under the License.

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_graceful_motion_controller/ego_polar_coords.hpp"

TEST(EgocentricPolarCoordinatesTest, ConstructorDefault) {
  nav2_graceful_motion_controller::EgocentricPolarCoordinates coords;

  EXPECT_DOUBLE_EQ(0.0, coords.r);
  EXPECT_DOUBLE_EQ(0.0, coords.phi);
  EXPECT_DOUBLE_EQ(0.0, coords.delta);
}

TEST(EgocentricPolarCoordinatesTest, ConstructorWithValues) {
  double r_value = 5.0;
  double phi_value = 1.2;
  double delta_value = -0.5;

  nav2_graceful_motion_controller::EgocentricPolarCoordinates coords(r_value, phi_value,
    delta_value);

  EXPECT_DOUBLE_EQ(r_value, coords.r);
  EXPECT_DOUBLE_EQ(phi_value, coords.phi);
  EXPECT_DOUBLE_EQ(delta_value, coords.delta);
}

TEST(EgocentricPolarCoordinatesTest, ConstructorFromPoses) {
  geometry_msgs::msg::Pose target;
  target.position.x = 3.0;
  target.position.y = 4.0;
  target.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0.8));

  geometry_msgs::msg::Pose current;
  current.position.x = 1.0;
  current.position.y = 1.0;
  current.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), -0.2));

  nav2_graceful_motion_controller::EgocentricPolarCoordinates coords(target, current);

  // Perform assertions based on expected values.

  EXPECT_DOUBLE_EQ(3.6055512428283691, coords.r);
  EXPECT_DOUBLE_EQ(-0.18279374837875384, coords.phi);
  EXPECT_DOUBLE_EQ(-1.1827937483787536, coords.delta);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
