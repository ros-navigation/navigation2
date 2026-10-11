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
#include <string>
#include <vector>

#include "nav2_msgs/msg/classified_pose.hpp"
#include "nav2_path_classifier/classification_filter.hpp"
#include "rclcpp/logger.hpp"

namespace nav2_path_classifier
{

using nav2_msgs::msg::ClassifiedPose;

/**
 * @brief Build a classified path with one pose per letter, where 'A' is class 1, 'B' is 2, ...
 * @param classes Letters giving the class of each pose in order
 * @return Classified path with pose i at x = i
 */
static std::vector<ClassifiedPose> makePath(const std::string & classes)
{
  std::vector<ClassifiedPose> path(classes.size());
  for (size_t i = 0; i < classes.size(); ++i) {
    path[i].pose.pose.position.x = static_cast<double>(i);
    path[i].class_type = static_cast<uint16_t>(classes[i] - 'A' + 1);
  }
  return path;
}

/**
 * @brief Convert a classified path back to its class letters.
 * @param path Classified path
 * @return One letter per pose, where class 1 is 'A', 2 is 'B', ...
 */
static std::string toLetters(const std::vector<ClassifiedPose> & path)
{
  std::string letters;
  for (const auto & pose : path) {
    letters += static_cast<char>('A' + pose.class_type - 1);
  }
  return letters;
}

/**
 * @brief Filter a path given as class letters and return the filtered letters.
 * @param classes Letters giving the raw class of each pose
 * @param hysteresis_window Hysteresis window of the filter
 * @return Filtered class letters
 */
static std::string filterLetters(const std::string & classes, int hysteresis_window)
{
  ClassificationFilter filter(hysteresis_window, rclcpp::get_logger("test"));
  auto path = makePath(classes);
  filter.filter(path);
  return toLetters(path);
}

TEST(ClassificationFilterTest, EmptyPathIsNoOp)
{
  ClassificationFilter filter(3, rclcpp::get_logger("test"));
  std::vector<ClassifiedPose> path;
  filter.filter(path);
  EXPECT_TRUE(path.empty());
}

TEST(ClassificationFilterTest, SinglePoseUnchanged)
{
  EXPECT_EQ(filterLetters("B", 3), "B");
}

TEST(ClassificationFilterTest, AllSameClassUnchanged)
{
  EXPECT_EQ(filterLetters("AAAAAA", 3), "AAAAAA");
}

TEST(ClassificationFilterTest, RunShorterThanWindowSuppressed)
{
  EXPECT_EQ(filterLetters("AAAABBAAAA", 3), "AAAAAAAAAA");
}

TEST(ClassificationFilterTest, ConfirmedRunBackfilled)
{
  EXPECT_EQ(filterLetters("AAAABBBBAAAA", 3), "AAAABBBBAAAA");
}

TEST(ClassificationFilterTest, TrailingRunOfExactlyWindowConfirmed)
{
  EXPECT_EQ(filterLetters("AAAABBB", 3), "AAAABBB");
}

TEST(ClassificationFilterTest, UnconfirmedTrailingRunKeepsCurrentClass)
{
  EXPECT_EQ(filterLetters("AAAAABB", 3), "AAAAAAA");
}

TEST(ClassificationFilterTest, FlickerNeverConfirms)
{
  EXPECT_EQ(filterLetters("AAABBABBAAA", 3), "AAAAAAAAAAA");
}

TEST(ClassificationFilterTest, PosesBeforeFirstConfirmedClassTakeIt)
{
  EXPECT_EQ(filterLetters("AABBCCC", 3), "CCCCCCC");
}

TEST(ClassificationFilterTest, MultipleTransitions)
{
  EXPECT_EQ(filterLetters("AABBCCAA", 2), "AABBCCAA");
}

TEST(ClassificationFilterTest, WindowOnePassesClassesThrough)
{
  EXPECT_EQ(filterLetters("ABCABC", 1), "ABCABC");
}

TEST(ClassificationFilterTest, ShortFirstRunTakesNextClass)
{
  EXPECT_EQ(filterLetters("ABBBBB", 3), "BBBBBB");
  EXPECT_EQ(filterLetters("AABBBB", 3), "BBBBBB");
}

TEST(ClassificationFilterTest, FirstRunOfExactlyWindowKept)
{
  EXPECT_EQ(filterLetters("AAABBBB", 3), "AAABBBB");
}

TEST(ClassificationFilterTest, NoConfirmedTransitionKeepsFirstClass)
{
  EXPECT_EQ(filterLetters("AB", 3), "AA");
}

TEST(ClassificationFilterTest, WindowBelowOneClampedToOne)
{
  EXPECT_EQ(filterLetters("ABAB", 0), "ABAB");
  EXPECT_EQ(filterLetters("ABAB", -2), "ABAB");
}

TEST(ClassificationFilterTest, OnlyClassTypeIsModified)
{
  ClassificationFilter filter(3, rclcpp::get_logger("test"));
  auto path = makePath("ABBBBBAAAA");
  filter.filter(path);
  for (size_t i = 0; i < path.size(); ++i) {
    EXPECT_DOUBLE_EQ(path[i].pose.pose.position.x, static_cast<double>(i));
  }
}

}  // namespace nav2_path_classifier

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
