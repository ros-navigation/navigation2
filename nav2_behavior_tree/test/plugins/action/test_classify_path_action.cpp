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

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/action/classify_path_action.hpp"
#include "nav2_behavior_tree/utils/classified_path_markers.hpp"

using nav2_behavior_tree::ClassifyPath;
using nav2_msgs::msg::ClassifiedPose;
using visualization_msgs::msg::MarkerArray;
using Classes = std::vector<uint16_t>;
using Points = std::vector<std::pair<double, double>>;

static constexpr uint16_t FREE = ClassifiedPose::FREE_SPACE;
static constexpr uint16_t CONFINED = ClassifiedPose::CONFINED_SPACE;
static constexpr uint16_t LAST_MILE = ClassifiedPose::LAST_MILE;
static constexpr double kInf = std::numeric_limits<double>::infinity();

// Confined space only, in 0.1 m steps up to 0.5 m, with no filter
static const char kConfinedOnly[] =
  R"(inflation_resolution="0.1" max_constraint_clearance="0.5" use_last_mile="false" )"
  R"(use_classification_filter="false")";

// 10 m x 10 m free costmap centered on the origin at 0.05 m resolution
static nav2_msgs::msg::Costmap makeFreeCostmap(const std::string & frame = "map")
{
  nav2_msgs::msg::Costmap msg;
  msg.header.frame_id = frame;
  msg.metadata.size_x = 200;
  msg.metadata.size_y = 200;
  msg.metadata.resolution = 0.05;
  msg.metadata.origin.position.x = -5.0;
  msg.metadata.origin.position.y = -5.0;
  msg.data.resize(msg.metadata.size_x * msg.metadata.size_y, nav2_costmap_2d::FREE_SPACE);
  return msg;
}

// Mark cells whose center is in [x_min, x_max) x [y_min, y_max) as lethal
static void addWall(
  nav2_msgs::msg::Costmap & msg, double y_min, double y_max, double x_min, double x_max)
{
  const auto & meta = msg.metadata;
  for (unsigned int row = 0; row < meta.size_y; ++row) {
    const double y = meta.origin.position.y + (row + 0.5) * meta.resolution;
    for (unsigned int col = 0; col < meta.size_x; ++col) {
      const double x = meta.origin.position.x + (col + 0.5) * meta.resolution;
      if (y >= y_min && y < y_max && x >= x_min && x < x_max) {
        msg.data[row * meta.size_x + col] = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
    }
  }
}

// Free costmap with lethal walls on both sides of the x axis
static nav2_msgs::msg::Costmap makeCorridor(
  double inner, double outer, double x_min = -kInf, double x_max = kInf)
{
  auto msg = makeFreeCostmap();
  addWall(msg, inner, outer, x_min, x_max);
  addWall(msg, -outer, -inner, x_min, x_max);
  return msg;
}

// Path through the given points in the map frame, facing +x
static nav_msgs::msg::Path makePath(const Points & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

// Points along the x axis
static Points alongX(const std::vector<double> & xs)
{
  Points points;
  for (double x : xs) {
    points.emplace_back(x, 0.0);
  }
  return points;
}

// Class of each classified pose
static Classes classesOf(const std::vector<ClassifiedPose> & classified_path)
{
  Classes classes;
  for (const auto & classified_pose : classified_path) {
    classes.push_back(classified_pose.class_type);
  }
  return classes;
}

class ClassifyPathTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  static void SetUpTestCase()
  {
    BehaviorTreeTestFixture::SetUpTestCase();
    factory_->registerNodeType<ClassifyPath>("ClassifyPath");
  }

  void SetUp() override
  {
    // New topics per test, so a latched costmap from an earlier test never arrives
    static int test_count = 0;
    ++test_count;
    costmap_topic_ = "test_costmap_" + std::to_string(test_count);
    footprint_topic_ = "test_footprint_" + std::to_string(test_count);
    costmap_pub_ = node_->create_publisher<nav2_msgs::msg::Costmap>(
      costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    footprint_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
      footprint_topic_, rclcpp::SystemDefaultsQoS());

    config_->blackboard->set<nav_msgs::msg::Path>("path", nav_msgs::msg::Path());
    config_->blackboard->set<std::vector<ClassifiedPose>>("classified_path", {});
  }

  void TearDown() override
  {
    tree_.reset();
    footprint_msg_.reset();
    footprint_pub_.reset();
    costmap_pub_.reset();
  }

protected:
  // Create a tree with a single ClassifyPath node on this test's topics
  void createTree(const std::string & attributes)
  {
    const std::string xml_txt =
      R"(<root main_tree_to_execute="MainTree">
           <BehaviorTree ID="MainTree">
             <ClassifyPath path="{path}" classified_path="{classified_path}" costmap_topic=")" +
      costmap_topic_ + R"(" footprint_topic=")" + footprint_topic_ + R"(" )" + attributes +
      R"(/>
           </BehaviorTree>
         </root>)";
    tree_ = std::make_shared<BT::Tree>(
      factory_->createTreeFromText(xml_txt, config_->blackboard));
  }

  // Publish a footprint the way a costmap does, around the robot at the map origin
  void publishFootprint(const Points & vertices, const std::string & frame = "map")
  {
    footprint_msg_ = std::make_shared<geometry_msgs::msg::PolygonStamped>();
    footprint_msg_->header.frame_id = frame;
    for (const auto & vertex : vertices) {
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(vertex.first);
      point.y = static_cast<float>(vertex.second);
      footprint_msg_->polygon.points.push_back(point);
    }
    footprint_pub_->publish(*footprint_msg_);
  }

  // Publish a square footprint centered on the robot
  void publishSquareFootprint(double half_side)
  {
    publishFootprint(
      {{half_side, half_side}, {-half_side, half_side},
        {-half_side, -half_side}, {half_side, -half_side}});
  }

  // Set the path input
  void setPath(const nav_msgs::msg::Path & path)
  {
    config_->blackboard->set<nav_msgs::msg::Path>("path", path);
  }

  // Classes output by the last tick
  Classes outputClasses()
  {
    return classesOf(config_->blackboard->get<std::vector<ClassifiedPose>>("classified_path"));
  }

  // Tick until a success outputs classes accepted by done, republishing the footprint
  Classes classifyUntil(const std::function<bool(const Classes &)> & done)
  {
    Classes classes;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      if (footprint_msg_) {
        footprint_pub_->publish(*footprint_msg_);
      }
      if (tree_->rootNode()->executeTick() == BT::NodeStatus::SUCCESS) {
        classes = outputClasses();
        if (done(classes)) {
          break;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return classes;
  }

  // Tick until the first success
  Classes classifyOnceReady()
  {
    return classifyUntil([](const Classes &) {return true;});
  }

  // Tick until the first pose has the given class
  Classes classifyUntilFirstIs(uint16_t class_type)
  {
    return classifyUntil(
      [class_type](const Classes & classes) {
        return !classes.empty() && classes[0] == class_type;
      });
  }

  // Tick for a while, expecting every tick to fail with an empty output
  void expectFailureFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      if (footprint_msg_) {
        footprint_pub_->publish(*footprint_msg_);
      }
      EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::FAILURE);
      EXPECT_TRUE(outputClasses().empty());
      std::this_thread::sleep_for(50ms);
    }
  }

  std::string costmap_topic_;
  std::string footprint_topic_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_msg_;
  std::shared_ptr<BT::Tree> tree_;
};

TEST_F(ClassifyPathTestFixture, RejectsNonPositiveResolution)
{
  EXPECT_THROW(createTree(R"(inflation_resolution="0")"), std::runtime_error);
}

TEST_F(ClassifyPathTestFixture, RejectsInvalidLastMileParams)
{
  EXPECT_THROW(createTree(R"(last_mile_ratio="-0.1")"), std::runtime_error);
  EXPECT_THROW(createTree(R"(last_mile_ratio="1.1")"), std::runtime_error);
  EXPECT_THROW(createTree(R"(last_mile_distance="-1")"), std::runtime_error);
}

TEST_F(ClassifyPathTestFixture, FailsOnEmptyPath)
{
  createTree(R"(use_confined_space="false")");
  config_->blackboard->set<std::vector<ClassifiedPose>>(
    "classified_path", std::vector<ClassifiedPose>(1));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(outputClasses().empty());
}

TEST_F(ClassifyPathTestFixture, FailsWithoutCostmap)
{
  createTree(kConfinedOnly);
  setPath(makePath({{0.0, 0.0}}));
  publishSquareFootprint(0.3);
  expectFailureFor(500ms);
}

TEST_F(ClassifyPathTestFixture, NoConfinedSpaceWithoutFootprint)
{
  // Like the server, no footprint skips confined space, even without a costmap
  createTree(kConfinedOnly);
  setPath(makePath({{0.0, 0.0}}));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outputClasses(), Classes{FREE});
}

TEST_F(ClassifyPathTestFixture, FailsOnFrameMismatch)
{
  createTree(kConfinedOnly);
  costmap_pub_->publish(makeFreeCostmap("odom"));
  publishSquareFootprint(0.3);
  auto path = makePath({{0.0, 0.0}});
  setPath(path);
  expectFailureFor(500ms);

  // Same data with matching frames succeeds
  path.header.frame_id = "odom";
  setPath(path);
  EXPECT_EQ(classifyOnceReady(), Classes{FREE});
}

TEST_F(ClassifyPathTestFixture, ClassifiesEachPoseOfPath)
{
  createTree(kConfinedOnly);
  costmap_pub_->publish(makeCorridor(0.55, 0.85));
  publishSquareFootprint(0.3);

  // First pose is in the corridor, second is above the top wall in open space
  setPath(makePath({{0.0, 0.0}, {0.0, 3.0}}));
  EXPECT_EQ(classifyUntilFirstIs(CONFINED), (Classes{CONFINED, FREE}));
}

TEST_F(ClassifyPathTestFixture, LastMileWinsOverConfinedSpace)
{
  createTree(
    R"(inflation_resolution="0.1" max_constraint_clearance="0.5" last_mile_ratio="0.5" )"
    R"(last_mile_distance="0" use_classification_filter="false")");
  costmap_pub_->publish(makeCorridor(0.55, 0.85));
  publishSquareFootprint(0.3);

  // Every pose is in the corridor, the last 1.5 m of the 3 m path is last mile
  setPath(makePath(alongX({0.0, 1.0, 2.0, 3.0})));
  EXPECT_EQ(
    classifyUntilFirstIs(CONFINED), (Classes{CONFINED, CONFINED, LAST_MILE, LAST_MILE}));
}

TEST_F(ClassifyPathTestFixture, NoMatchForDegenerateFootprint)
{
  createTree(kConfinedOnly);
  costmap_pub_->publish(makeCorridor(0.55, 0.85));
  publishFootprint({{0.3, 0.3}, {-0.3, -0.3}});
  setPath(makePath({{0.0, 0.0}}));
  EXPECT_EQ(classifyOnceReady(), Classes{FREE});
}

TEST_F(ClassifyPathTestFixture, NewCostmapPickedUpOnNextTick)
{
  createTree(kConfinedOnly);
  costmap_pub_->publish(makeCorridor(0.55, 0.85));
  publishSquareFootprint(0.3);
  setPath(makePath({{0.0, 0.0}}));
  EXPECT_EQ(classifyUntilFirstIs(CONFINED), Classes{CONFINED});

  costmap_pub_->publish(makeFreeCostmap());
  EXPECT_EQ(classifyUntilFirstIs(FREE), Classes{FREE});
}

TEST_F(ClassifyPathTestFixture, FootprintChangeIsPickedUp)
{
  createTree(kConfinedOnly);
  costmap_pub_->publish(makeCorridor(0.75, 1.0));
  setPath(makePath({{0.0, 0.0}}));

  // Edge reaches 0.3 + 0.5 = 0.8 m, inside walls starting at 0.75 m
  publishSquareFootprint(0.3);
  EXPECT_EQ(classifyUntilFirstIs(CONFINED), Classes{CONFINED});

  // Edge reaches 0.1 + 0.5 = 0.6 m, short of the walls
  publishSquareFootprint(0.1);
  EXPECT_EQ(classifyUntilFirstIs(FREE), Classes{FREE});
}

TEST_F(ClassifyPathTestFixture, SkipsConfinedSpaceWhenTfFails)
{
  createTree(kConfinedOnly);
  costmap_pub_->publish(makeCorridor(0.55, 0.85));
  publishSquareFootprint(0.3);
  setPath(makePath({{0.0, 0.0}}));
  EXPECT_EQ(classifyUntilFirstIs(CONFINED), Classes{CONFINED});

  // Like the server, a footprint with no transform to the robot skips confined space
  publishFootprint({{0.3, 0.3}, {-0.3, 0.3}, {-0.3, -0.3}, {0.3, -0.3}}, "unknown_frame");
  EXPECT_EQ(classifyUntilFirstIs(FREE), Classes{FREE});
}

TEST_F(ClassifyPathTestFixture, SwitchesTurnStepsOff)
{
  // Nothing is published, and the path is shorter than last_mile_distance
  createTree(
    R"(use_confined_space="false" use_last_mile="false" last_mile_distance="10")");
  setPath(makePath(alongX({0.0, 1.0})));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outputClasses(), (Classes{FREE, FREE}));
}

TEST_F(ClassifyPathTestFixture, ThresholdMarksDistanceToGoal)
{
  createTree(
    R"(use_confined_space="false" last_mile_ratio="0" last_mile_distance="0.75" )"
    R"(use_classification_filter="false")");

  // Poses within 0.75 m of the goal along a single 2 m plan
  setPath(makePath(alongX({0.0, 0.5, 1.0, 1.5, 2.0})));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outputClasses(), (Classes{FREE, FREE, FREE, LAST_MILE, LAST_MILE}));

  // Shorter than last_mile_distance, so the full path is last mile
  setPath(makePath(alongX({0.0, 0.25, 0.5})));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outputClasses(), Classes(3, LAST_MILE));
}

TEST_F(ClassifyPathTestFixture, LastMileByArcLength)
{
  createTree(
    R"(use_confined_space="false" last_mile_ratio="0.5" last_mile_distance="2.0" )"
    R"(use_classification_filter="false")");

  // 20 poses over the first 5 m and 6 over the last 5 m, so splitting by length marks the last 6
  std::vector<double> xs;
  for (int i = 0; i < 20; ++i) {
    xs.push_back(0.25 * i);
  }
  for (int i = 5; i <= 10; ++i) {
    xs.push_back(i);
  }
  setPath(makePath(alongX(xs)));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  const auto classes = outputClasses();
  ASSERT_EQ(classes.size(), xs.size());
  for (size_t i = 0; i < xs.size(); ++i) {
    EXPECT_EQ(classes[i], xs[i] >= 5.0 ? LAST_MILE : FREE) << "pose " << i << " at x = " << xs[i];
  }
}

TEST_F(ClassifyPathTestFixture, FilterSuppressesShortBlip)
{
  // Walls only at x in [0, 0.1) confine the 3 poses within 0.5 m of them
  costmap_pub_->publish(makeCorridor(0.55, 0.85, 0.0, 0.1));
  setPath(makePath(alongX({-3.0, -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0})));

  createTree(kConfinedOnly);
  publishSquareFootprint(0.3);
  const auto raw = classifyUntil(
    [](const Classes & classes) {
      return std::count(classes.begin(), classes.end(), CONFINED) > 0;
    });
  EXPECT_EQ(std::count(raw.begin(), raw.end(), CONFINED), 3);

  // A window of 4 suppresses the 3 pose blip
  tree_.reset();
  createTree(
    R"(inflation_resolution="0.1" max_constraint_clearance="0.5" use_last_mile="false" )"
    R"(hysteresis_window="4")");
  EXPECT_EQ(classifyOnceReady(), Classes(11, FREE));
}

TEST_F(ClassifyPathTestFixture, NoClassifiedMarkersWhenFilterDisabled)
{
  createTree(
    R"(use_confined_space="false" use_classification_filter="false" )"
    R"(publish_classified_path="true")");

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (node_->count_publishers("raw_classified_path_markers") == 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_GT(node_->count_publishers("raw_classified_path_markers"), 0u);
  EXPECT_EQ(node_->count_publishers("classified_path_markers"), 0u);
}

TEST_F(ClassifyPathTestFixture, PublishesRawAndClassifiedMarkers)
{
  std::mutex mutex;
  std::vector<MarkerArray> raw, classified;
  auto raw_sub = node_->create_subscription<MarkerArray>(
    "raw_classified_path_markers", 10,
    [&](MarkerArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex);
      raw.push_back(*msg);
    });
  auto classified_sub = node_->create_subscription<MarkerArray>(
    "classified_path_markers", 10,
    [&](MarkerArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex);
      classified.push_back(*msg);
    });

  createTree(
    R"(use_confined_space="false" last_mile_ratio="0.5" last_mile_distance="0" )"
    R"(hysteresis_window="1" publish_classified_path="true")");

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while ((raw_sub->get_publisher_count() == 0 || classified_sub->get_publisher_count() == 0) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(20ms);
  }

  // The last 1.5 m of the 3 m path is last mile
  const auto path = makePath(alongX({0.0, 1.0, 2.0, 3.0}));
  const Classes expected{FREE, FREE, LAST_MILE, LAST_MILE};
  setPath(path);
  ASSERT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);

  deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!raw.empty() && !classified.empty()) {
        break;
      }
    }
    std::this_thread::sleep_for(20ms);
  }

  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_FALSE(raw.empty());
  ASSERT_FALSE(classified.empty());

  // Raw: DELETEALL plus one sphere per pose
  ASSERT_EQ(raw.front().markers.size(), path.poses.size() + 1);

  // Classified: DELETEALL plus one line strip colored per pose
  ASSERT_EQ(classified.front().markers.size(), 2u);
  const auto & strip = classified.front().markers[1];
  ASSERT_EQ(strip.colors.size(), path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto color = nav2_behavior_tree::utils::colorForClass(expected[i]);
    EXPECT_FLOAT_EQ(strip.colors[i].r, color.r) << "pose " << i;
    EXPECT_FLOAT_EQ(strip.colors[i].g, color.g) << "pose " << i;
    EXPECT_FLOAT_EQ(strip.colors[i].b, color.b) << "pose " << i;
  }
}

// Classified path with one pose per letter, where 'A' is class 1, 'B' is 2, ...
static std::vector<ClassifiedPose> makeLetterPath(const std::string & classes)
{
  std::vector<ClassifiedPose> path(classes.size());
  for (size_t i = 0; i < classes.size(); ++i) {
    path[i].pose.pose.position.x = static_cast<double>(i);
    path[i].class_type = static_cast<uint16_t>(classes[i] - 'A' + 1);
  }
  return path;
}

// Filter a path given as class letters
static std::string filterLetters(const std::string & classes, int hysteresis_window)
{
  auto path = makeLetterPath(classes);
  ClassifyPath::filterClasses(path, hysteresis_window);
  std::string letters;
  for (const auto & pose : path) {
    letters += static_cast<char>('A' + pose.class_type - 1);
  }
  return letters;
}

TEST(ClassificationFilterTest, EmptyPathIsNoOp)
{
  std::vector<ClassifiedPose> path;
  ClassifyPath::filterClasses(path, 3);
  EXPECT_TRUE(path.empty());
}

TEST(ClassificationFilterTest, RunShorterThanWindowSuppressed)
{
  EXPECT_EQ(filterLetters("AAAABBAAAA", 3), "AAAAAAAAAA");
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
  auto path = makeLetterPath("ABBBBBAAAA");
  ClassifyPath::filterClasses(path, 3);
  for (size_t i = 0; i < path.size(); ++i) {
    EXPECT_DOUBLE_EQ(path[i].pose.pose.position.x, static_cast<double>(i));
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
