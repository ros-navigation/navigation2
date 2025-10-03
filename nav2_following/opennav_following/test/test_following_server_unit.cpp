// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#include <chrono>
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"
#include "opennav_following/following_server.hpp"
#include "nav2_ros_common/node_thread.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

// Testing unit functions in following server, smoke/system tests in python file

using namespace std::chrono_literals;  // NOLINT

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_following
{

using FollowObject = nav2_msgs::action::FollowObject;

class FollowingServerShim : public FollowingServer
{
public:
  FollowingServerShim()
  : FollowingServer() {}

  void setUsingDedicatedThread()
  {
    tf2_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  }

  virtual bool approachObject(geometry_msgs::msg::PoseStamped &, const std::string &)
  {
    std::string exception;
    this->get_parameter("exception_to_throw", exception);
    if (exception == "TransformException") {
      throw tf2::TransformException("TransformException");
    } else if (exception == "FailedToDetectObject") {
      throw opennav_docking_core::FailedToDetectDock("FailedToDetectObject");
    } else if (exception == "FailedToControl") {
      throw opennav_docking_core::FailedToControl("FailedToControl");
    } else if (exception == "DockingException") {
      throw opennav_docking_core::DockingException("DockingException");
    } else if (exception == "exception") {
      throw std::exception();
    }
    return true;
  }

  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
  {
    return FollowingServer::getRefinedPose(pose);
  }

  virtual bool getFramePose(geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id)
  {
    return FollowingServer::getFramePose(pose, frame_id);
  }

  virtual bool getTrackingPose(geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id)
  {
    return FollowingServer::getTrackingPose(pose, frame_id);
  }

  virtual bool rotateToObject(geometry_msgs::msg::PoseStamped &)
  {
    return true;
  }

  geometry_msgs::msg::PoseStamped getPoseAtDistance(
    const geometry_msgs::msg::PoseStamped & pose, double distance)
  {
    return FollowingServer::getPoseAtDistance(pose, distance);
  }

  bool isGoalReached(const geometry_msgs::msg::PoseStamped & goal_pose)
  {
    return FollowingServer::isGoalReached(goal_pose);
  }

  void setDynamicPose(const geometry_msgs::msg::PoseStamped & pose)
  {
    detected_dynamic_pose_ = pose;
  }

  void setSkipOrientation(bool skip_orientation)
  {
    skip_orientation_ = skip_orientation;
  }

  void setFixedFrame(const std::string & fixed_frame)
  {
    fixed_frame_ = fixed_frame;
  }
};

TEST(FollowingServerTests, ObjectLifecycle)
{
  auto node = std::make_shared<opennav_following::FollowingServer>();
  node->configure();
  node->activate();
  node->deactivate();
  node->cleanup();
  node->shutdown();
  node.reset();
}

TEST(FollowingServerTests, ErrorExceptions)
{
  auto node = std::make_shared<FollowingServerShim>();
  auto node_thread = nav2::NodeThread(node);
  auto node2 = std::make_shared<rclcpp::Node>("client_node");

  auto pub = node2->create_publisher<geometry_msgs::msg::PoseStamped>(
    "dynamic_pose", rclcpp::QoS(1));

  geometry_msgs::msg::PoseStamped detected_pose;

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  node->declare_parameter("exception_to_throw", rclcpp::ParameterValue(""));
  node->declare_parameter("follow_action_called", rclcpp::ParameterValue(false));
  node->set_parameter(rclcpp::Parameter("fixed_frame", rclcpp::ParameterValue("test_frame")));

  // Error codes following
  std::vector<std::string> error_ids{
    "TransformException", "FailedToDetectObject", "FailedToControl",
    "DockingException", "exception"};
  std::vector<int> error_codes{901, 902, 903, 999, 999};

  // Call action, check error code
  for (unsigned int i = 0; i != error_ids.size(); i++) {
    node->set_parameter(
      rclcpp::Parameter("exception_to_throw", rclcpp::ParameterValue(error_ids[i])));

    auto client = rclcpp_action::create_client<FollowObject>(node2, "follow_object");
    if (!client->wait_for_action_server(1s)) {
      RCLCPP_ERROR(node2->get_logger(), "Action server not available after waiting");
    }
    auto goal_msg = FollowObject::Goal();
    goal_msg.pose_topic = "dynamic_pose";
    auto future_goal_handle = client->async_send_goal(goal_msg);
    pub->publish(detected_pose);

    if (rclcpp::spin_until_future_complete(
        node2, future_goal_handle, 2s) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto future_result = client->async_get_result(future_goal_handle.get());
      if (rclcpp::spin_until_future_complete(
          node2, future_result, 5s) == rclcpp::FutureReturnCode::SUCCESS)
      {
        auto result = future_result.get();
        EXPECT_EQ(result.result->error_code, error_codes[i]);
      } else {
        EXPECT_TRUE(false);
      }
    } else {
      EXPECT_TRUE(false);
    }
  }

  // Set follow_action_called to true to simulate robot following object
  node->set_parameter(rclcpp::Parameter("follow_action_called", true));

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(FollowingServerTests, GetPoseAtDistance)
{
  auto node = std::make_shared<opennav_following::FollowingServerShim>();
  node->set_parameter(rclcpp::Parameter("base_frame", rclcpp::ParameterValue("my_frame")));
  node->on_configure(rclcpp_lifecycle::State());
  node->setUsingDedicatedThread();

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node->now();
  pose.header.frame_id = "my_frame";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = -1.0;

  auto new_pose = node->getPoseAtDistance(pose, 0.2);
  EXPECT_NEAR(new_pose.pose.position.x, 0.8585, 0.01);
  EXPECT_NEAR(new_pose.pose.position.y, -0.8585, 0.01);

  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(FollowingServerTests, IsGoalReached)
{
  auto node = std::make_shared<opennav_following::FollowingServerShim>();
  node->set_parameter(rclcpp::Parameter("base_frame", rclcpp::ParameterValue("my_frame")));
  node->on_configure(rclcpp_lifecycle::State());
  node->setUsingDedicatedThread();

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node->now();
  pose.header.frame_id = "my_frame";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = -1.0;

  EXPECT_FALSE(node->isGoalReached(pose));

  // Set the pose below the tolerance
  pose.pose.position.x = 0.1;
  pose.pose.position.y = 0.1;
  EXPECT_TRUE(node->isGoalReached(pose));

  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(FollowingServerTests, RefinedPose)
{
  auto node = std::make_shared<opennav_following::FollowingServerShim>();

  // Set filter coefficient to 0, so no filtering is done
  node->set_parameter(rclcpp::Parameter("filter_coef", rclcpp::ParameterValue(0.0)));
  node->set_parameter(rclcpp::Parameter("base_frame", rclcpp::ParameterValue("my_frame")));
  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  // Timestamps are outdated; this is after timeout
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_FALSE(node->getRefinedPose(pose));

  // Set skip orientation to false
  node->setSkipOrientation(false);

  // Set the detected pose
  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "my_frame";
  detected_pose.pose.position.x = 0.1;
  detected_pose.pose.position.y = -0.1;
  node->setDynamicPose(detected_pose);

  node->setFixedFrame("my_frame");
  EXPECT_TRUE(node->getRefinedPose(pose));
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.1, 0.01);

  // Now, set skip orientation to true
  node->setSkipOrientation(true);

  detected_pose.header.stamp = node->now();
  node->setDynamicPose(detected_pose);

  EXPECT_TRUE(node->getRefinedPose(pose));
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.1, 0.01);

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(FollowingServerTests, GetFramePose)
{
  auto node = std::make_shared<opennav_following::FollowingServerShim>();

  // Set filter coefficient to 0, so no filtering is done
  node->set_parameter(rclcpp::Parameter("filter_coef", rclcpp::ParameterValue(0.0)));

  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped pose;

  // Not frame set, should return false
  auto frame_test = std::string("my_frame");
  node->setFixedFrame("fixed_frame_test");
  EXPECT_FALSE(node->getFramePose(pose, frame_test));

  // Set transform between my_frame and fixed_frame_test
  auto tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped frame_to_fixed;
  frame_to_fixed.header.frame_id = "fixed_frame_test";
  frame_to_fixed.header.stamp = node->get_clock()->now();
  frame_to_fixed.child_frame_id = "my_frame";
  frame_to_fixed.transform.translation.x = 1.0;
  frame_to_fixed.transform.translation.y = 2.0;
  frame_to_fixed.transform.translation.z = 3.0;
  tf_broadcaster->sendTransform(frame_to_fixed);

  // Now, we should be able to get the pose in my_frame
  EXPECT_TRUE(node->getFramePose(pose, frame_test));
  EXPECT_EQ(pose.pose.position.x, 1.0);
  EXPECT_EQ(pose.pose.position.y, 2.0);
  EXPECT_EQ(pose.pose.position.z, 3.0);

  node->on_deactivate(rclcpp_lifecycle::State());
  node->on_cleanup(rclcpp_lifecycle::State());
  node->on_shutdown(rclcpp_lifecycle::State());
  node.reset();
}

TEST(FollowingServerTests, DynamicParams)
{
  auto node = std::make_shared<opennav_following::FollowingServer>();
  node->on_configure(rclcpp_lifecycle::State());
  node->on_activate(rclcpp_lifecycle::State());

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("controller_frequency", 1.0),
      rclcpp::Parameter("detection_timeout", 2.0),
      rclcpp::Parameter("rotate_to_object_timeout", 3.0),
      rclcpp::Parameter("static_object_timeout", 4.0),
      rclcpp::Parameter("desired_distance", 5.0),
      rclcpp::Parameter("linear_tolerance", 6.0),
      rclcpp::Parameter("angular_tolerance", 7.0),
      rclcpp::Parameter("base_frame", std::string("test_base_frame")),
      rclcpp::Parameter("fixed_frame", std::string("test_fixed_frame")),
      rclcpp::Parameter("skip_orientation", false),
      rclcpp::Parameter("search_by_rotating", true),
      rclcpp::Parameter("search_angle", 8.0),
      rclcpp::Parameter("transform_tolerance", 9.0)
    });

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("controller_frequency").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("detection_timeout").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("rotate_to_object_timeout").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("static_object_timeout").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("desired_distance").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("linear_tolerance").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("angular_tolerance").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("base_frame").as_string(), "test_base_frame");
  EXPECT_EQ(node->get_parameter("fixed_frame").as_string(), "test_fixed_frame");
  EXPECT_EQ(node->get_parameter("skip_orientation").as_bool(), false);
  EXPECT_EQ(node->get_parameter("search_by_rotating").as_bool(), true);
  EXPECT_EQ(node->get_parameter("search_angle").as_double(), 8.0);
  EXPECT_EQ(node->get_parameter("transform_tolerance").as_double(), 9.0);
}

}  // namespace opennav_following
