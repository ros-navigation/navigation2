// Copyright (c) 2026 Origin
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

#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_path_classifier/classifiers/last_mile_classifier.hpp"

namespace nav2_path_classifier
{

using nav2_msgs::msg::ClassifiedPose;

/**
 * @brief Build a path along the x axis.
 * @param xs X positions of the poses in metres
 * @return Path with one pose per x position
 */
static nav_msgs::msg::Path makePath(const std::vector<double> & xs)
{
  nav_msgs::msg::Path path;
  for (double x : xs) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

class LastMileClassifierTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    node_ = std::make_shared<nav2_util::LifecycleNode>("last_mile_classifier_test");
  }

  void TearDown() override
  {
    node_.reset();
  }

  /**
   * @brief Configure the classifier under a parameter namespace.
   * @param name Parameter namespace of the classifier instance
   */
  void configure(const std::string & name)
  {
    classifier_.configure(node_, name, nullptr, nullptr, nullptr);
  }

  /**
   * @brief Declare the parameters of a classifier instance.
   * @param name Parameter namespace of the classifier instance
   * @param ratio last_mile_ratio value
   * @param threshold claim_full_path_threshold value
   */
  void declareParams(const std::string & name, double ratio, double threshold)
  {
    node_->declare_parameter(name + ".class_type", ClassifiedPose::LAST_MILE);
    node_->declare_parameter(name + ".last_mile_ratio", ratio);
    node_->declare_parameter(name + ".claim_full_path_threshold", threshold);
  }

  nav2_util::LifecycleNode::SharedPtr node_;
  LastMileClassifier classifier_;
};

TEST_F(LastMileClassifierTest, ConfigureRejectsInvalidParams)
{
  EXPECT_ANY_THROW(configure("missing"));

  declareParams("low_ratio", -0.1, 2.0);
  EXPECT_THROW(configure("low_ratio"), std::runtime_error);

  declareParams("high_ratio", 1.1, 2.0);
  EXPECT_THROW(configure("high_ratio"), std::runtime_error);

  declareParams("negative_threshold", 0.5, -1.0);
  EXPECT_THROW(configure("negative_threshold"), std::runtime_error);
}

TEST_F(LastMileClassifierTest, ClassifiesByArcLength)
{
  declareParams("lastmile", 0.5, 2.0);
  configure("lastmile");

  EXPECT_TRUE(classifier_.classify(nav_msgs::msg::Path()).empty());

  // Shorter than claim_full_path_threshold, so the full path is last mile
  const auto short_result = classifier_.classify(makePath({0.0, 0.5, 1.0, 1.5}));
  ASSERT_EQ(short_result.size(), 4u);
  for (const auto & classified_pose : short_result) {
    EXPECT_EQ(classified_pose.class_type, ClassifiedPose::LAST_MILE);
  }

  // 10 m path with 20 poses over the first 5 m and 6 over the last 5 m. Splitting
  // by pose count would mark 13 poses, splitting by length marks the last 6
  std::vector<double> xs;
  for (int i = 0; i < 20; ++i) {
    xs.push_back(0.25 * i);
  }
  for (int i = 5; i <= 10; ++i) {
    xs.push_back(i);
  }
  const auto result = classifier_.classify(makePath(xs));
  ASSERT_EQ(result.size(), xs.size());
  for (size_t i = 0; i < result.size(); ++i) {
    const uint16_t expected = xs[i] >= 5.0 ? ClassifiedPose::LAST_MILE : ClassifiedPose::UNKNOWN;
    EXPECT_EQ(result[i].class_type, expected) << "pose " << i << " at x = " << xs[i];
  }
}

}  // namespace nav2_path_classifier

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
