// Copyright (c) 2021 RoboTech Vision
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

#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_core/smoother.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_smoother/nav2_smoother.hpp"

using SmoothAction = nav2_msgs::action::SmoothPath;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<SmoothAction>;

using namespace std::chrono_literals;

// A smoother for testing the base class

class DummySmoother : public nav2_core::Smoother
{
public:
  DummySmoother() {}

  ~DummySmoother() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) {}

  virtual void cleanup() {}

  virtual void activate() {}

  virtual void deactivate() {}

  virtual bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time)
  {
    assert(path.poses.size() == 2);

    if (path.poses.front() == path.poses.back()) {
      throw nav2_core::PlannerException("Start and goal pose must differ");
    }

    auto max_time_ms = max_time.to_chrono<std::chrono::milliseconds>();
    std::this_thread::sleep_for(std::min(max_time_ms, 100ms));

    // place dummy pose in the middle of the path
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x =
      (path.poses.front().pose.position.x + path.poses.back().pose.position.x) / 2;
    pose.pose.position.y =
      (path.poses.front().pose.position.y + path.poses.back().pose.position.y) / 2;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);

    return max_time_ms > 100ms;
  }

private:
  std::string command_;
  std::chrono::system_clock::time_point start_time_;
};

// Mocked class loader
void onPluginDeletion(nav2_core::Smoother * obj)
{
  if (nullptr != obj) {
    delete (obj);
  }
}

template<>
pluginlib::UniquePtr<nav2_core::Smoother> pluginlib::ClassLoader<nav2_core::Smoother>::
createUniqueInstance(const std::string & lookup_name)
{
  if (lookup_name != "DummySmoother") {
    // original method body
    if (!isClassLoaded(lookup_name)) {
      loadLibraryForClass(lookup_name);
    }
    try {
      std::string class_type = getClassType(lookup_name);
      pluginlib::UniquePtr<nav2_core::Smoother> obj =
        lowlevel_class_loader_.createUniqueInstance<nav2_core::Smoother>(class_type);
      return obj;
    } catch (const class_loader::CreateClassException & ex) {
      throw pluginlib::CreateClassException(ex.what());
    }
  }

  // mocked plugin creation
  return std::unique_ptr<nav2_core::Smoother,
           class_loader::ClassLoader::DeleterType<nav2_core::Smoother>>(
    new DummySmoother(),
    onPluginDeletion);
}

class DummyCostmapSubscriber : public nav2_costmap_2d::CostmapSubscriber
{
public:
  DummyCostmapSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic_name)
  : CostmapSubscriber(node, topic_name)
  {
    auto costmap = std::make_shared<nav2_msgs::msg::Costmap>();
    costmap->metadata.size_x = 100;
    costmap->metadata.size_y = 100;
    costmap->metadata.resolution = 0.1;
    costmap->metadata.origin.position.x = -5.0;
    costmap->metadata.origin.position.y = -5.0;

    costmap->data.resize(costmap->metadata.size_x * costmap->metadata.size_y, 0);
    for (unsigned int i = 0; i < costmap->metadata.size_y; ++i) {
      for (unsigned int j = 20; j < 40; ++j) {
        costmap->data[i * costmap->metadata.size_x + j] = 254;
      }
    }

    setCostmap(costmap);
  }

  void setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    costmap_msg_ = msg;
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
      msg->metadata.size_x, msg->metadata.size_y,
      msg->metadata.resolution, msg->metadata.origin.position.x,
      msg->metadata.origin.position.y);

    processCurrentCostmapMsg();
  }
};

class DummyFootprintSubscriber : public nav2_costmap_2d::FootprintSubscriber
{
public:
  DummyFootprintSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic_name,
    tf2_ros::Buffer & tf)
  : FootprintSubscriber(node, topic_name, tf)
  {
    auto footprint = std::make_shared<geometry_msgs::msg::PolygonStamped>();
    footprint->header.frame_id = "base_link";  // global frame = robot frame to avoid tf lookup
    footprint->header.stamp = node->get_clock()->now();
    geometry_msgs::msg::Point32 point;
    point.x = -0.2f;
    point.y = -0.2f;
    footprint->polygon.points.push_back(point);
    point.y = 0.2f;
    footprint->polygon.points.push_back(point);
    point.x = 0.2f;
    point.y = 0.0f;
    footprint->polygon.points.push_back(point);

    setFootprint(footprint);
  }

  void setFootprint(geometry_msgs::msg::PolygonStamped::SharedPtr msg)
  {
    footprint_ = msg;
    footprint_received_ = true;
  }
};

class DummySmootherServer : public nav2_smoother::SmootherServer
{
public:
  DummySmootherServer()
  {
    // Override defaults
    default_ids_.clear();
    default_ids_.resize(1, "SmoothPath");
    set_parameter(rclcpp::Parameter("smoother_plugins", default_ids_));
    default_types_.clear();
    default_types_.resize(1, "DummySmoother");
  }

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state)
  {
    auto result = SmootherServer::on_configure(state);
    if (result != nav2_util::CallbackReturn::SUCCESS) {
      return result;
    }

    // Create dummy subscribers and collision checker
    auto node = shared_from_this();
    costmap_sub_ =
      std::make_shared<DummyCostmapSubscriber>(
      node, "costmap_topic");
    footprint_sub_ =
      std::make_shared<DummyFootprintSubscriber>(
      node, "footprint_topic", *tf_);
    collision_checker_ =
      std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
      *costmap_sub_, *footprint_sub_,
      node->get_name());

    return result;
  }
};

// Define a test class to hold the context for the tests
class SmootherTest : public ::testing::Test
{
public:
  SmootherTest() {}
  ~SmootherTest() {}

  void SetUp() override
  {
    node_ =
      std::make_shared<rclcpp::Node>(
      "LifecycleSmootherTestNode", rclcpp::NodeOptions());

    smoother_server_ = std::make_shared<DummySmootherServer>();
    smoother_server_->set_parameter(
      rclcpp::Parameter(
        "smoother_plugins",
        rclcpp::ParameterValue(std::vector<std::string>(1, "DummySmoothPath"))));
    smoother_server_->declare_parameter(
      "DummySmoothPath.plugin",
      rclcpp::ParameterValue(std::string("DummySmoother")));
    smoother_server_->configure();
    smoother_server_->activate();

    client_ = rclcpp_action::create_client<SmoothAction>(
      node_->get_node_base_interface(),
      node_->get_node_graph_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_waitables_interface(), "smooth_path");
    std::cout << "Setup complete." << std::endl;
  }

  void TearDown() override
  {
    smoother_server_->deactivate();
    smoother_server_->cleanup();
    smoother_server_->shutdown();
    smoother_server_.reset();
    client_.reset();
    node_.reset();
  }

  bool sendGoal(
    std::string smoother_id, double x_start, double y_start, double x_goal,
    double y_goal, std::chrono::milliseconds max_time, bool check_for_collisions)
  {
    if (!client_->wait_for_action_server(4s)) {
      std::cout << "Server not up" << std::endl;
      return false;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.orientation.w = 1.0;

    auto goal = SmoothAction::Goal();
    goal.smoother_id = smoother_id;
    pose.pose.position.x = x_start;
    pose.pose.position.y = y_start;
    goal.path.poses.push_back(pose);
    pose.pose.position.x = x_goal;
    pose.pose.position.y = y_goal;
    goal.path.poses.push_back(pose);
    goal.check_for_collisions = check_for_collisions;
    goal.max_smoothing_duration = rclcpp::Duration(max_time);

    auto future_goal = client_->async_send_goal(goal);

    if (rclcpp::spin_until_future_complete(node_, future_goal) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cout << "failed sending goal" << std::endl;
      // failed sending the goal
      return false;
    }

    goal_handle_ = future_goal.get();

    if (!goal_handle_) {
      std::cout << "goal was rejected" << std::endl;
      // goal was rejected by the action server
      return false;
    }

    return true;
  }

  ClientGoalHandle::WrappedResult getResult()
  {
    std::cout << "Getting async result..." << std::endl;
    auto future_result = client_->async_get_result(goal_handle_);
    std::cout << "Waiting on future..." << std::endl;
    rclcpp::spin_until_future_complete(node_, future_result);
    std::cout << "future received!" << std::endl;
    return future_result.get();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<DummySmootherServer> smoother_server_;
  std::shared_ptr<rclcpp_action::Client<SmoothAction>> client_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<SmoothAction>> goal_handle_;
};

// Define the tests

TEST_F(SmootherTest, testingSuccess)
{
  ASSERT_TRUE(sendGoal("DummySmoothPath", 0.0, 0.0, 1.0, 0.0, 500ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.result->path.poses.size(), (std::size_t)3);
  EXPECT_TRUE(result.result->was_completed);
  SUCCEED();
}

TEST_F(SmootherTest, testingFailureOnInvalidSmootherId)
{
  ASSERT_TRUE(sendGoal("InvalidSmoother", 0.0, 0.0, 1.0, 0.0, 500ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  SUCCEED();
}

TEST_F(SmootherTest, testingSuccessOnEmptyPlugin)
{
  ASSERT_TRUE(sendGoal("", 0.0, 0.0, 1.0, 0.0, 500ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  SUCCEED();
}

TEST_F(SmootherTest, testingIncomplete)
{
  ASSERT_TRUE(sendGoal("DummySmoothPath", 0.0, 0.0, 1.0, 0.0, 50ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FALSE(result.result->was_completed);
  SUCCEED();
}

TEST_F(SmootherTest, testingFailureOnException)
{
  ASSERT_TRUE(sendGoal("DummySmoothPath", 0.0, 0.0, 0.0, 0.0, 500ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  SUCCEED();
}

TEST_F(SmootherTest, testingFailureOnCollision)
{
  ASSERT_TRUE(sendGoal("DummySmoothPath", -4.0, 0.0, 0.0, 0.0, 500ms, true));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  SUCCEED();
}

TEST_F(SmootherTest, testingCollisionCheckDisabled)
{
  ASSERT_TRUE(sendGoal("DummySmoothPath", -4.0, 0.0, 0.0, 0.0, 500ms, false));
  auto result = getResult();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  SUCCEED();
}

TEST(SmootherConfigTest, testingConfigureSuccessWithValidSmootherPlugin)
{
  auto smoother_server = std::make_shared<DummySmootherServer>();
  smoother_server->set_parameter(
    rclcpp::Parameter(
      "smoother_plugins",
      rclcpp::ParameterValue(std::vector<std::string>(1, "DummySmoothPath"))));
  smoother_server->declare_parameter(
    "DummySmoothPath.plugin",
    rclcpp::ParameterValue(std::string("DummySmoother")));
  auto state = smoother_server->configure();
  EXPECT_EQ(state.id(), 2);  // 1 on failure, 2 on success
  SUCCEED();
}

TEST(SmootherConfigTest, testingConfigureFailureWithInvalidSmootherPlugin)
{
  auto smoother_server = std::make_shared<DummySmootherServer>();
  smoother_server->set_parameter(
    rclcpp::Parameter(
      "smoother_plugins",
      rclcpp::ParameterValue(std::vector<std::string>(1, "DummySmoothPath"))));
  smoother_server->declare_parameter(
    "DummySmoothPath.plugin",
    rclcpp::ParameterValue(std::string("InvalidSmootherPlugin")));
  auto state = smoother_server->configure();
  EXPECT_EQ(state.id(), 1);  // 1 on failure, 2 on success
  SUCCEED();
}

TEST(SmootherConfigTest, testingConfigureSuccessWithDefaultPlugin)
{
  auto smoother_server = std::make_shared<DummySmootherServer>();
  auto state = smoother_server->configure();
  EXPECT_EQ(state.id(), 2);  // 1 on failure, 2 on success
  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
