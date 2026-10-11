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
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_msgs/action/classify_path.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_path_classifier/path_classifier.hpp"
#include "nav2_path_classifier/path_classifier_server.hpp"
#include "nav2_path_classifier/utils.hpp"

using namespace std::chrono_literals;

namespace nav2_path_classifier
{

using ClassifyPath = nav2_msgs::action::ClassifyPath;
using ClassifiedPose = nav2_msgs::msg::ClassifiedPose;
using MarkerArray = visualization_msgs::msg::MarkerArray;

/**
 * @class RangeClassifier
 * @brief Test classifier that matches poses whose x lies in [x_min, x_max).
 */
class RangeClassifier : public PathClassifier
{
public:
  /**
   * @brief A constructor for RangeClassifier
   * @param x_min Lower x bound of the matched range
   * @param x_max Upper x bound of the matched range
   * @param class_type Class reported for matched poses
   * @param throw_at_x If not NaN, classify() throws when the path has a pose at this x
   * @param wrong_size If true, classify() returns one pose fewer than the path
   */
  RangeClassifier(
    double x_min, double x_max, uint16_t class_type, double throw_at_x = std::nan(""),
    bool wrong_size = false)
  : x_min_(x_min), x_max_(x_max), class_type_(class_type), throw_at_x_(throw_at_x),
    wrong_size_(wrong_size)
  {
  }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override {}

  void cleanup() override {++cleanup_count;}

  void activate() override {++activate_count;}

  void deactivate() override {++deactivate_count;}

  std::vector<ClassifiedPose> classify(const nav_msgs::msg::Path & path) override
  {
    std::vector<ClassifiedPose> classified_path(path.poses.size());
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const double x = path.poses[i].pose.position.x;
      if (!std::isnan(throw_at_x_) && std::fabs(x - throw_at_x_) < 1e-9) {
        throw std::runtime_error("RangeClassifier: requested failure");
      }
      classified_path[i].pose = path.poses[i];
      classified_path[i].class_type =
        (x >= x_min_ && x < x_max_) ? class_type_ : ClassifiedPose::UNKNOWN;
    }
    if (wrong_size_ && !classified_path.empty()) {
      classified_path.pop_back();
    }
    return classified_path;
  }

  int activate_count{0};
  int deactivate_count{0};
  int cleanup_count{0};

private:
  double x_min_;
  double x_max_;
  uint16_t class_type_;
  double throw_at_x_;
  bool wrong_size_;
};

/**
 * @class DummyPathClassifierServer
 * @brief Server that appends test classifiers after the configured plugins are loaded.
 */
class DummyPathClassifierServer : public PathClassifierServer
{
public:
  std::vector<PathClassifier::Ptr> injected_classifiers;

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    auto result = PathClassifierServer::on_configure(state);
    classifiers_.insert(
      classifiers_.end(), injected_classifiers.begin(), injected_classifiers.end());
    return result;
  }
};

/**
 * @brief Build a straight path along x from 0 to 14.5 m in 0.5 m steps (30 poses).
 * @return Path in the map frame
 */
static nav_msgs::msg::Path makeStraightPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < 30; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.5 * i;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

/**
 * @brief Expected class at x for the four-class setup built by fourClassifiers().
 * @param x Pose x in metres
 * @return Expected class, where class 2 wins the overlap at [4, 5)
 */
static uint16_t fourClassExpected(double x)
{
  if (x >= 2.0 && x < 5.0) {
    return 2;
  }
  if (x >= 5.0 && x < 8.0) {
    return 3;
  }
  if (x >= 8.0 && x < 11.0) {
    return 4;
  }
  return 1;
}

class PathClassifierServerTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("path_classifier_server_test_client");
    client_ = rclcpp_action::create_client<ClassifyPath>(node_, "classify_path");
  }

  void TearDown() override
  {
    stopServer();
    client_.reset();
    node_.reset();
  }

  /**
   * @brief Bring the server down from whatever state it is in and destroy it.
   */
  void stopServer()
  {
    if (!server_) {
      return;
    }
    if (server_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      server_->deactivate();
    }
    if (server_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      server_->cleanup();
    }
    server_->shutdown();
    server_.reset();
  }

  /**
   * @brief Three range classifiers giving classes 2, 3 and 4, with 2 and 3 overlapping at [4, 5).
   * @return Classifiers in priority order
   */
  std::vector<std::shared_ptr<RangeClassifier>> fourClassifiers()
  {
    return {
      std::make_shared<RangeClassifier>(2.0, 5.0, 2),
      std::make_shared<RangeClassifier>(4.0, 8.0, 3),
      std::make_shared<RangeClassifier>(8.0, 11.0, 4)};
  }

  /**
   * @brief Create the server with the given settings, then configure it.
   * @param classifiers Test classifiers appended after any configured plugins
   * @param use_filter Value of use_classification_filter
   * @param publish Value of publish_classified_path
   * @return Lifecycle state id after configuring
   */
  uint8_t configureServer(
    const std::vector<std::shared_ptr<RangeClassifier>> & classifiers,
    bool use_filter, bool publish)
  {
    server_ = std::make_shared<DummyPathClassifierServer>();
    server_->set_parameter(rclcpp::Parameter("use_classification_filter", use_filter));
    server_->set_parameter(rclcpp::Parameter("publish_classified_path", publish));
    server_->set_parameter(rclcpp::Parameter("classification_filter.hysteresis_window", 4));
    for (const auto & classifier : classifiers) {
      server_->injected_classifiers.push_back(classifier);
    }
    return server_->configure().id();
  }

  /**
   * @brief Create, configure and activate the server.
   * @param classifiers Test classifiers appended after any configured plugins
   * @param use_filter Value of use_classification_filter
   * @param publish Value of publish_classified_path
   */
  void startServer(
    const std::vector<std::shared_ptr<RangeClassifier>> & classifiers,
    bool use_filter, bool publish)
  {
    ASSERT_EQ(
      configureServer(classifiers, use_filter, publish),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(server_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  /**
   * @brief Send a goal and wait for its result.
   * @param path Path to classify
   * @return Wrapped action result
   */
  rclcpp_action::ClientGoalHandle<ClassifyPath>::WrappedResult classify(
    const nav_msgs::msg::Path & path)
  {
    EXPECT_TRUE(client_->wait_for_action_server(5s));
    ClassifyPath::Goal goal;
    goal.path = path;
    auto goal_future = client_->async_send_goal(goal);
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(node_, goal_future, 5s),
      rclcpp::FutureReturnCode::SUCCESS);
    auto goal_handle = goal_future.get();
    EXPECT_TRUE(goal_handle);
    auto result_future = client_->async_get_result(goal_handle);
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(node_, result_future, 5s),
      rclcpp::FutureReturnCode::SUCCESS);
    return result_future.get();
  }

  /**
   * @brief Spin the client node until the condition holds or the timeout expires.
   * @param condition Condition to wait for
   * @param timeout Maximum time to wait
   * @return true if the condition held before the timeout
   */
  bool spinUntil(const std::function<bool()> & condition, std::chrono::seconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (condition()) {
        return true;
      }
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(10ms);
    }
    return condition();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ClassifyPath>::SharedPtr client_;
  std::shared_ptr<DummyPathClassifierServer> server_;
};

TEST_F(PathClassifierServerTest, NoClassifiersGivesDefaultClass)
{
  startServer({}, false, false);
  const auto path = makeStraightPath();
  const auto result = classify(path);

  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_EQ(result.result->classified_path.size(), path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    EXPECT_EQ(result.result->classified_path[i].class_type, ClassifiedPose::FREE_SPACE);
    EXPECT_DOUBLE_EQ(
      result.result->classified_path[i].pose.pose.position.x, path.poses[i].pose.position.x);
  }
}

TEST_F(PathClassifierServerTest, EmptyPathIsAborted)
{
  startServer(fourClassifiers(), true, false);
  const auto result = classify(nav_msgs::msg::Path());
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(PathClassifierServerTest, ClassifiesFourClassesPerPose)
{
  startServer(fourClassifiers(), false, false);
  const auto path = makeStraightPath();
  const auto result = classify(path);

  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_EQ(result.result->classified_path.size(), path.poses.size());
  std::set<uint16_t> seen;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double x = path.poses[i].pose.position.x;
    const auto & classified = result.result->classified_path[i];
    EXPECT_EQ(classified.class_type, fourClassExpected(x)) << "pose at x = " << x;
    EXPECT_DOUBLE_EQ(classified.pose.pose.position.x, x);
    seen.insert(classified.class_type);
  }
  EXPECT_EQ(seen, (std::set<uint16_t>{1, 2, 3, 4}));
  EXPECT_GE(rclcpp::Duration(result.result->classification_time).nanoseconds(), 0);
}

TEST_F(PathClassifierServerTest, FirstMatchingClassifierWins)
{
  startServer(fourClassifiers(), false, false);
  const auto result = classify(makeStraightPath());

  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  // Poses at x = 4.0 and 4.5 match both the class 2 and class 3 classifiers
  EXPECT_EQ(result.result->classified_path[8].class_type, 2u);
  EXPECT_EQ(result.result->classified_path[9].class_type, 2u);
}

TEST_F(PathClassifierServerTest, FilterKeepsRunsAtLeastWindowLong)
{
  // Every run in the four-class setup is at least 4 poses long, so nothing changes
  startServer(fourClassifiers(), true, false);
  const auto path = makeStraightPath();
  const auto result = classify(path);

  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  for (size_t i = 0; i < path.poses.size(); ++i) {
    EXPECT_EQ(
      result.result->classified_path[i].class_type,
      fourClassExpected(path.poses[i].pose.position.x));
  }
}

TEST_F(PathClassifierServerTest, FilterSuppressesShortBlip)
{
  // A 2 pose blip of class 4 inside a class 3 stretch, shorter than the window of 4
  auto blip = std::make_shared<RangeClassifier>(4.0, 5.0, 4);
  auto stretch = std::make_shared<RangeClassifier>(0.0, 10.0, 3);
  const auto path = makeStraightPath();

  startServer({blip, stretch}, false, false);
  const auto raw = classify(path);
  ASSERT_EQ(raw.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(raw.result->classified_path[8].class_type, 4u);
  EXPECT_EQ(raw.result->classified_path[9].class_type, 4u);
  stopServer();

  startServer({blip, stretch}, true, false);
  const auto filtered = classify(path);
  ASSERT_EQ(filtered.code, rclcpp_action::ResultCode::SUCCEEDED);
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const uint16_t expected = path.poses[i].pose.position.x < 10.0 ? 3 : 1;
    EXPECT_EQ(filtered.result->classified_path[i].class_type, expected) << "pose " << i;
  }
}

TEST_F(PathClassifierServerTest, ClassifierExceptionAbortsGoal)
{
  auto failing = std::make_shared<RangeClassifier>(2.0, 5.0, 2, 3.0);
  startServer({failing}, true, false);
  const auto result = classify(makeStraightPath());
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(PathClassifierServerTest, WrongSizeResultAbortsGoal)
{
  auto broken = std::make_shared<RangeClassifier>(2.0, 5.0, 2, std::nan(""), true);
  startServer({broken}, true, false);
  const auto result = classify(makeStraightPath());
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(PathClassifierServerTest, LifecycleReachesEveryClassifier)
{
  auto classifiers = fourClassifiers();
  startServer(classifiers, true, false);
  for (const auto & classifier : classifiers) {
    EXPECT_EQ(classifier->activate_count, 1);
  }

  server_->deactivate();
  for (const auto & classifier : classifiers) {
    EXPECT_EQ(classifier->deactivate_count, 1);
  }

  server_->cleanup();
  for (const auto & classifier : classifiers) {
    EXPECT_EQ(classifier->cleanup_count, 1);
  }
}

TEST_F(PathClassifierServerTest, PublishesRawAndFilteredMarkers)
{
  std::vector<MarkerArray::SharedPtr> raw, filtered;
  auto raw_sub = node_->create_subscription<MarkerArray>(
    "raw_classified_path_markers", 10,
    [&raw](MarkerArray::SharedPtr msg) {raw.push_back(msg);});
  auto filtered_sub = node_->create_subscription<MarkerArray>(
    "filtered_classified_path_markers", 10,
    [&filtered](MarkerArray::SharedPtr msg) {filtered.push_back(msg);});

  startServer(fourClassifiers(), true, true);
  ASSERT_TRUE(
    spinUntil(
      [&]() {
        return raw_sub->get_publisher_count() > 0 && filtered_sub->get_publisher_count() > 0;
      }));

  const auto path = makeStraightPath();
  ASSERT_EQ(classify(path).code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(spinUntil([&]() {return !raw.empty() && !filtered.empty();}));

  // Raw: DELETEALL plus one sphere per pose
  ASSERT_EQ(raw.front()->markers.size(), path.poses.size() + 1);

  // Filtered: DELETEALL plus one line strip colored per pose
  ASSERT_EQ(filtered.front()->markers.size(), 2u);
  const auto & strip = filtered.front()->markers[1];
  ASSERT_EQ(strip.colors.size(), path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto expected = utils::colorForClass(fourClassExpected(path.poses[i].pose.position.x));
    EXPECT_FLOAT_EQ(strip.colors[i].r, expected.r);
    EXPECT_FLOAT_EQ(strip.colors[i].g, expected.g);
    EXPECT_FLOAT_EQ(strip.colors[i].b, expected.b);
  }
}

TEST_F(PathClassifierServerTest, NoFilteredMarkersWithoutFilter)
{
  std::vector<MarkerArray::SharedPtr> raw;
  auto raw_sub = node_->create_subscription<MarkerArray>(
    "raw_classified_path_markers", 10,
    [&raw](MarkerArray::SharedPtr msg) {raw.push_back(msg);});
  auto filtered_sub = node_->create_subscription<MarkerArray>(
    "filtered_classified_path_markers", 10, [](MarkerArray::SharedPtr) {});

  startServer(fourClassifiers(), false, true);
  ASSERT_TRUE(spinUntil([&]() {return raw_sub->get_publisher_count() > 0;}));

  ASSERT_EQ(classify(makeStraightPath()).code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(spinUntil([&]() {return !raw.empty();}));
  EXPECT_EQ(filtered_sub->get_publisher_count(), 0u);
}

TEST_F(PathClassifierServerTest, LoadsRealPluginByName)
{
  server_ = std::make_shared<DummyPathClassifierServer>();
  server_->set_parameter(
    rclcpp::Parameter("pose_classifier_plugins", std::vector<std::string>{"Confined"}));
  server_->declare_parameter(
    "Confined.plugin", std::string("nav2_path_classifier/ConfinedSpaceClassifier"));
  server_->declare_parameter("Confined.class_type", 2);
  EXPECT_EQ(server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(PathClassifierServerTest, UnknownPluginFailsConfigure)
{
  server_ = std::make_shared<DummyPathClassifierServer>();
  server_->set_parameter(
    rclcpp::Parameter("pose_classifier_plugins", std::vector<std::string>{"Bad"}));
  server_->declare_parameter("Bad.plugin", std::string("nav2_path_classifier/NotAClassifier"));
  EXPECT_NE(server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
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
