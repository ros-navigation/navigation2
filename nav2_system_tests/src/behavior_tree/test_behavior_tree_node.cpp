// Copyright (c) 2020 Sarthak Mittal
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

#include <vector>
#include <string>
#include <fstream>
#include <filesystem>
#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/odometry_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "server_handler.hpp"

using namespace std::chrono_literals;

namespace nav2_system_tests
{

class BehaviorTreeHandler
{
public:
  BehaviorTreeHandler()
  {
    node_ = rclcpp::Node::make_shared("behavior_tree_handler");

    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(), node_->get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    tf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, node_, false);

    odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node_);

    const std::vector<std::string> plugin_libs = {
      "nav2_compute_path_to_pose_action_bt_node",
      "nav2_compute_path_through_poses_action_bt_node",
      "nav2_smooth_path_action_bt_node",
      "nav2_follow_path_action_bt_node",
      "nav2_spin_action_bt_node",
      "nav2_wait_action_bt_node",
      "nav2_assisted_teleop_action_bt_node",
      "nav2_back_up_action_bt_node",
      "nav2_drive_on_heading_bt_node",
      "nav2_clear_costmap_service_bt_node",
      "nav2_is_stuck_condition_bt_node",
      "nav2_goal_reached_condition_bt_node",
      "nav2_initial_pose_received_condition_bt_node",
      "nav2_goal_updated_condition_bt_node",
      "nav2_globally_updated_goal_condition_bt_node",
      "nav2_is_path_valid_condition_bt_node",
      "nav2_are_error_codes_active_condition_bt_node",
      "nav2_would_a_controller_recovery_help_condition_bt_node",
      "nav2_would_a_planner_recovery_help_condition_bt_node",
      "nav2_would_a_smoother_recovery_help_condition_bt_node",
      "nav2_reinitialize_global_localization_service_bt_node",
      "nav2_rate_controller_bt_node",
      "nav2_distance_controller_bt_node",
      "nav2_speed_controller_bt_node",
      "nav2_truncate_path_action_bt_node",
      "nav2_truncate_path_local_action_bt_node",
      "nav2_goal_updater_node_bt_node",
      "nav2_recovery_node_bt_node",
      "nav2_pipeline_sequence_bt_node",
      "nav2_round_robin_node_bt_node",
      "nav2_transform_available_condition_bt_node",
      "nav2_time_expired_condition_bt_node",
      "nav2_path_expiring_timer_condition",
      "nav2_distance_traveled_condition_bt_node",
      "nav2_single_trigger_bt_node",
      "nav2_is_battery_low_condition_bt_node",
      "nav2_navigate_through_poses_action_bt_node",
      "nav2_navigate_to_pose_action_bt_node",
      "nav2_remove_passed_goals_action_bt_node",
      "nav2_planner_selector_bt_node",
      "nav2_controller_selector_bt_node",
      "nav2_goal_checker_selector_bt_node",
      "nav2_controller_cancel_bt_node",
      "nav2_path_longer_on_approach_bt_node",
      "nav2_assisted_teleop_cancel_bt_node",
      "nav2_wait_cancel_bt_node",
      "nav2_spin_cancel_bt_node",
      "nav2_back_up_cancel_bt_node",
      "nav2_drive_on_heading_cancel_bt_node",
      "nav2_goal_updated_controller_bt_node"
    };
    for (const auto & p : plugin_libs) {
      factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
    }
  }

  bool loadBehaviorTree(const std::string & filename)
  {
    // Read the input BT XML from the specified file into a string
    std::ifstream xml_file(filename);

    if (!xml_file.good()) {
      RCLCPP_ERROR(node_->get_logger(), "Couldn't open input XML file: %s", filename.c_str());
      return false;
    }

    auto xml_string = std::string(
      std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

    // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard = BT::Blackboard::create();

    // Put items on the blackboard
    blackboard->set<rclcpp::Node::SharedPtr>("node", node_);  // NOLINT
    blackboard->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(20));  // NOLINT
    blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration", std::chrono::milliseconds(10));  // NOLINT
    blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout", std::chrono::milliseconds(1000));  // NOLINT
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);  // NOLINT
    blackboard->set<bool>("initial_pose_received", false);  // NOLINT
    blackboard->set<int>("number_recoveries", 0);  // NOLINT
    blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother_);  // NOLINT

    // set dummy goal on blackboard
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = "map";
    goal.pose.position.x = 0.0;
    goal.pose.position.y = 0.0;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal);  // NOLINT

    // Create the Behavior Tree from the XML input
    try {
      tree = factory_.createTreeFromText(xml_string, blackboard);
    } catch (BT::RuntimeError & exp) {
      RCLCPP_ERROR(node_->get_logger(), "%s: %s", filename.c_str(), exp.what());
      return false;
    }

    return true;
  }

  std::string generateBTLogFileName()
  {
    auto end = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::string time_str = std::ctime(&end_time);
    std::replace(time_str.begin(), time_str.end(), ' ', '_');
    time_str.erase(std::remove(time_str.begin(), time_str.end(), '\n'), time_str.end());
    std::string file_name = "bt_trace_" + time_str + "_.fbl";
    return file_name;
  }

public:
  BT::Blackboard::Ptr blackboard;
  BT::Tree tree;

private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  BT::BehaviorTreeFactory factory_;
};

class BehaviorTreeTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    // initialize ROS
    rclcpp::init(0, nullptr);

    server_handler = std::make_shared<ServerHandler>();
    if (!server_handler->isActive()) {
      server_handler->activate();
    }
  }

  static void TearDownTestCase()
  {
    // shutdown ROS
    rclcpp::shutdown();

    server_handler.reset();
    bt_handler.reset();
  }

  void SetUp() override
  {
    server_handler->reset();
    bt_handler = std::make_shared<BehaviorTreeHandler>();
  }

  void TearDown() override
  {
    bt_handler.reset();
  }

protected:
  static std::shared_ptr<ServerHandler> server_handler;
  static std::shared_ptr<BehaviorTreeHandler> bt_handler;
};

std::shared_ptr<ServerHandler> BehaviorTreeTestFixture::server_handler = nullptr;
std::shared_ptr<BehaviorTreeHandler> BehaviorTreeTestFixture::bt_handler = nullptr;

TEST_F(BehaviorTreeTestFixture, TestBTXMLFiles)
{
  std::filesystem::path root = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
  root /= "behavior_trees/";

  if (std::filesystem::exists(root) && std::filesystem::is_directory(root)) {
    for (auto const & entry : std::filesystem::recursive_directory_iterator(root)) {
      if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ".xml") {
        std::cout << entry.path().string() << std::endl;
        EXPECT_EQ(bt_handler->loadBehaviorTree(entry.path().string()), true);
      }
    }
  }
}

/**
 * Test scenario:
 *
 * ComputePathToPose and FollowPath return SUCCESS
 * The behavior tree should execute correctly and return SUCCESS
 */
TEST_F(BehaviorTreeTestFixture, TestAllSuccess)
{
  // Load behavior tree from file
  std::filesystem::path bt_file = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
  bt_file /= "behavior_trees/";
  bt_file /= "navigate_to_pose_w_replanning_and_recovery.xml";
  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string()), true);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  while (result == BT::NodeStatus::RUNNING) {
    result = bt_handler->tree.tickRoot();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success since all action servers returned success
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

  // Goal count should be 1 since only one goal is sent to ComputePathToPose and FollowPath servers
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 1);
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 1);

  // Goal count should be 0 since no goal is sent to all other servers
  EXPECT_EQ(server_handler->spin_server->getGoalCount(), 0);
  EXPECT_EQ(server_handler->wait_server->getGoalCount(), 0);
  EXPECT_EQ(server_handler->backup_server->getGoalCount(), 0);
  EXPECT_EQ(server_handler->clear_local_costmap_server->getRequestCount(), 0);
  EXPECT_EQ(server_handler->clear_global_costmap_server->getRequestCount(), 0);
}

/**
 * Test scenario:
 *
 * ComputePathToPose returns FAILURE and ClearGlobalCostmap-Context returns FAILURE
 * PipelineSequence returns FAILURE and NavigateRecovery triggers RecoveryFallback
 * GoalUpdated returns FAILURE and RoundRobin is triggered
 * RoundRobin triggers ClearingActions Sequence which returns FAILURE
 * RoundRobin triggers Spin, Wait, and BackUp which return FAILURE
 * RoundRobin returns FAILURE hence RecoveryCallbackk returns FAILURE
 * Finally NavigateRecovery returns FAILURE
 * The behavior tree should return FAILURE
 */
TEST_F(BehaviorTreeTestFixture, TestAllFailure)
{
  // Load behavior tree from file
  std::filesystem::path bt_file = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
  bt_file /= "behavior_trees/";
  bt_file /= "navigate_to_pose_w_replanning_and_recovery.xml";
  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string()), true);

  // Set all action server to fail the first 100 times
  Ranges failureRange;
  failureRange.emplace_back(Range(0, 100));
  server_handler->compute_path_to_pose_server->setFailureRanges(failureRange);
  server_handler->follow_path_server->setFailureRanges(failureRange);
  server_handler->spin_server->setFailureRanges(failureRange);
  server_handler->wait_server->setFailureRanges(failureRange);
  server_handler->backup_server->setFailureRanges(failureRange);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  while (result == BT::NodeStatus::RUNNING) {
    result = bt_handler->tree.tickRoot();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be failure
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);

  // Goal count should be 2 since only two goals are sent to ComputePathToPose
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 14);

  // Goal count should be 0 since no goal is sent to FollowPath action server
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 0);

  EXPECT_EQ(server_handler->spin_server->getGoalCount(), 5);
  EXPECT_EQ(server_handler->wait_server->getGoalCount(), 5);
  EXPECT_EQ(server_handler->backup_server->getGoalCount(), 5);

  // Service count is 1 to try and resolve global planner error
  EXPECT_EQ(server_handler->clear_global_costmap_server->getRequestCount(), 13);

  EXPECT_EQ(server_handler->clear_local_costmap_server->getRequestCount(), 6);
}

/**
 * Test scenario:
 *
 * ComputePathToPose returns FAILURE on the first try triggering the planner recovery
 * ClearGlobalCostmap-Context returns SUCCESS and ComputePathToPose returns SUCCESS when retried
 * FollowPath returns FAILURE on the first try triggering the controller recovery
 * ClearLocalCostmap-Context returns SUCCESS and FollowPath returns SUCCESS when retried
 * The behavior tree should return SUCCESS
 */
TEST_F(BehaviorTreeTestFixture, TestNavigateSubtreeRecoveries)
{
  // Load behavior tree from file
  std::filesystem::path bt_file = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
  bt_file /= "behavior_trees/";
  bt_file /= "navigate_to_pose_w_replanning_and_recovery.xml";
  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string()), true);

  // Set ComputePathToPose and FollowPath action servers to fail for the first action
  Ranges failureRange;
  failureRange.emplace_back(Range(0, 1));
  server_handler->compute_path_to_pose_server->setFailureRanges(failureRange);
  server_handler->follow_path_server->setFailureRanges(failureRange);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  while (result == BT::NodeStatus::RUNNING) {
    result = bt_handler->tree.tickRoot();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

  // Goal count should be 2 since only two goals were sent to ComputePathToPose and FollowPath
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 2);
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 2);

  // Navigate subtree recovery services are called once each
  EXPECT_EQ(server_handler->clear_local_costmap_server->getRequestCount(), 1);
  EXPECT_EQ(server_handler->clear_global_costmap_server->getRequestCount(), 1);

  // Goal count should be 0 since no goal is sent to all other servers
  EXPECT_EQ(server_handler->spin_server->getGoalCount(), 0);
  EXPECT_EQ(server_handler->wait_server->getGoalCount(), 0);
  EXPECT_EQ(server_handler->backup_server->getGoalCount(), 0);
}

/**
 * Test scenario:
 *
 * ComputePathToPose returns FAILURE on the first try triggering the planner recovery
 * ClearGlobalCostmap-Context returns SUCCESS and ComputePathToPose returns SUCCESS when retried
 * FollowPath returns FAILURE on the first try triggering the controller recovery
 * ClearLocalCostmap-Context returns SUCCESS and FollowPath is retried
 * FollowPath returns FAILURE again and PipelineSequence returns FAILURE
 * NavigateRecovery triggers RecoveryFallback and GoalUpdated returns FAILURE
 * RoundRobin triggers ClearingActions Sequence which returns SUCCESS
 * RoundRobin returns SUCCESS and RecoveryFallback returns SUCCESS
 * PipelineSequence is triggered again and ComputePathToPose returns SUCCESS
 * FollowPath returns FAILURE on the third try triggering the controller recovery
 * ClearLocalCostmap-Context returns SUCCESS and FollowPath returns SUCCESS on the fourth try
 * The behavior tree should return SUCCESS
 */
TEST_F(BehaviorTreeTestFixture, TestNavigateRecoverySimple)
{
  // Load behavior tree from file
  std::filesystem::path bt_file = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
  bt_file /= "behavior_trees/";
  bt_file /= "navigate_to_pose_w_replanning_and_recovery.xml";
  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string()), true);

  // Set ComputePathToPose action server to fail for the first action
  Ranges plannerFailureRange;
  plannerFailureRange.emplace_back(Range(0, 1));
  server_handler->compute_path_to_pose_server->setFailureRanges(plannerFailureRange);

  // Set FollowPath action server to fail for the first 3 actions
  Ranges controllerFailureRange;
  controllerFailureRange.emplace_back(Range(0, 3));
  server_handler->follow_path_server->setFailureRanges(controllerFailureRange);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  while (result == BT::NodeStatus::RUNNING) {
    result = bt_handler->tree.tickRoot();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

  // FollowPath is called 4 times
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 4);

  // ComputePathToPose is called 3 times
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 3);

  // Local costmap is cleared 3 times
  EXPECT_EQ(server_handler->clear_local_costmap_server->getRequestCount(), 3);

  // Global costmap is cleared 2 times
  EXPECT_EQ(server_handler->clear_global_costmap_server->getRequestCount(), 2);

  // Goal count should be 0 since only no goal is sent to all other servers
  EXPECT_EQ(server_handler->spin_server->getGoalCount(), 0);
  EXPECT_EQ(server_handler->wait_server->getGoalCount(), 0);
  EXPECT_EQ(server_handler->backup_server->getGoalCount(), 0);
}

/**
 * Test Scenario:
 *
 * PipelineSequence returns FAILURE and triggers the Recovery subtree
 * NavigateRecovery ticks the recovery subtree, WouldAControllerRecoveryHelp returns SUCCESS
 * GoalUpdated returns FAILURE, RoundRobin triggers ClearingActions Sequence which returns SUCCESS
 * RoundRobin returns SUCCESS and the recovery subtree returns SUCCESS
 *
 * RETRY 1
 * PipelineSequence returns FAILURE and triggers the Recovery subtree
 * NavigateRecovery ticks the recovery subtree, WouldAControllerRecoveryHelp returns SUCCESS
 * GoalUpdated returns FAILURE, RoundRobin triggers Spin which returns FAILURE
 * RoundRobin triggers Wait which returns SUCCESS
 * RoundRobin returns SUCCESS and RecoveryFallback returns SUCCESS
 *
 * RETRY 2
 * PipelineSequence returns FAILURE and triggers the Recovery subtree
 * NavigateRecovery ticks the recovery subtree, WouldAControllerRecoveryHelp returns SUCCESS
 * GoalUpdated returns FAILURE and RoundRobin triggers BackUp which returns FAILURE
 * RoundRobin triggers ClearingActions Sequence which returns SUCCESS
 * RoundRobin returns SUCCESS and RecoveryFallback returns SUCCESS
 *
 * RETRY 3
 * PipelineSequence returns FAILURE and triggers the Recovery subtree
 * NavigateRecovery ticks the recovery subtree, WouldAControllerRecoveryHelp returns SUCCESS
 * GoalUpdated returns FAILURE and RoundRobin triggers ClearingActions which returns SUCCESS
 * RoundRobin returns SUCCESS and RecoveryFallback returns SUCCESS
 *
 * RETRY 4
 * PipelineSequence returns FAILURE and triggers the Recovery subtree
 * NavigateRecovery ticks the recovery subtree, WouldAControllerRecoveryHelp returns SUCCESS
 * WouldAControllerRecoveryHelp returns SUCCESS
 * GoalUpdated returns FAILURE and RoundRobin triggers Spin which returns SUCCESS
 *
 * RETRY 5
 * PipelineSequence returns FAILURE and triggers the Recovery subtree
 * NavigateRecovery ticks the recovery subtree, WouldAControllerRecoveryHelp returns SUCCESS
 * GoalUpdated returns FAILURE and RoundRobin triggers Wait which returns SUCCESS
 * RoundRobin triggers BackUp which returns SUCCESS
 *
 * RETRY 6
 * ComputePathToPose returns FAILURE on the first try triggering the planner recovery
 * ClearGlobalCostmap-Context returns SUCCESS and ComputePathToPose returns FAILURE when retried
 * PipelineSequence returns FAILURE and NavigateRecovery finally also returns FAILURE
 *
 * The behavior tree should return FAILURE
 *
 */
TEST_F(BehaviorTreeTestFixture, TestNavigateRecoveryComplex)
{
  // Load behavior tree from file
  std::filesystem::path bt_file = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
  bt_file /= "behavior_trees/";
  bt_file /= "navigate_to_pose_w_replanning_and_recovery.xml";
  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string()), true);

  // Set FollowPath action server to fail for the first 2 actions
  Ranges controllerFailureRange;
  controllerFailureRange.emplace_back(Range(0, 14));
  server_handler->follow_path_server->setFailureRanges(controllerFailureRange);

  // Set Spin action server to fail for the first action
  Ranges spinFailureRange;
  spinFailureRange.emplace_back(Range(0, 1));
  server_handler->spin_server->setFailureRanges(spinFailureRange);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  while (result == BT::NodeStatus::RUNNING) {
    result = bt_handler->tree.tickRoot();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);

  // ComputePathToPose is called 12 times
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 7);

  // FollowPath is called 4 times
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 14);

  // Local costmap is cleared 5 times
  EXPECT_EQ(server_handler->clear_local_costmap_server->getRequestCount(), 9);

  // Global costmap is cleared 8 times
  EXPECT_EQ(server_handler->clear_global_costmap_server->getRequestCount(), 2);

  // All recovery action servers receive 2 goals
  EXPECT_EQ(server_handler->spin_server->getGoalCount(), 2);
  EXPECT_EQ(server_handler->wait_server->getGoalCount(), 2);
  EXPECT_EQ(server_handler->backup_server->getGoalCount(), 1);
}

/**
 * Test scenario:
 *
 * The PipelineSequence return FAILURE due to FollowPath returning FAILURE
 * The NavigateRecovery triggers the recovery sub tree which returns SUCCESS
 * The PipelineSequence return FAILURE due to FollowPath returning FAILURE
 * The NavigateRecovery triggers the recovery sub tree which ticks the Spin recovery
 *
 * At this point a new goal is updated on the blackboard
 *
 * RecoveryFallback triggers GoalUpdated which returns SUCCESS this time
 * Since GoalUpdated returned SUCCESS, RoundRobin and hence Spin is halted
 * RecoveryFallback also returns SUCCESS and PipelineSequence is retried
 * PipelineSequence triggers ComputePathToPose which returns SUCCESS
 * FollowPath returns SUCCESS and NavigateRecovery finally also returns SUCCESS
 *
 * The behavior tree should return SUCCESS
 */
TEST_F(BehaviorTreeTestFixture, TestRecoverySubtreeGoalUpdated)
{
  // Load behavior tree from file
  std::filesystem::path bt_file = ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
  bt_file /= "behavior_trees/";
  bt_file /= "navigate_to_pose_w_replanning_and_recovery.xml";
  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string()), true);

  // Set FollowPath action server to fail for the first 2 actions
  Ranges controllerFailureRange;
  controllerFailureRange.emplace_back(Range(0, 4));
  server_handler->follow_path_server->setFailureRanges(controllerFailureRange);

  // Set Spin action server to return running for the first action
  Ranges spinRunningRange;
  spinRunningRange.emplace_back(Range(1, 1));
  server_handler->spin_server->setRunningRanges(spinRunningRange);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  while (result == BT::NodeStatus::RUNNING) {
    result = bt_handler->tree.tickRoot();

    // Update goal on blackboard after Spin has been triggered once
    // to simulate a goal update during a recovery action
    if (server_handler->spin_server->getGoalCount() > 0) {
      geometry_msgs::msg::PoseStamped goal;
      goal.pose.position.x = 1.0;
      goal.pose.position.y = 1.0;
      goal.pose.position.z = 1.0;
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = 0.0;
      goal.pose.orientation.w = 1.0;
      bt_handler->blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal);  // NOLINT
    }

    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

  // ComputePathToPose is called 4 times
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 3);

  // FollowPath is called 3 times
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 5);

  // Local costmap is cleared 2 times
  EXPECT_EQ(server_handler->clear_local_costmap_server->getRequestCount(), 3);

  // Global costmap is cleared 2 times
  EXPECT_EQ(server_handler->clear_global_costmap_server->getRequestCount(), 1);

  // Spin server receives 1 action
  EXPECT_EQ(server_handler->spin_server->getGoalCount(), 1);

  // All recovery action servers receive 0 goals
  EXPECT_EQ(server_handler->wait_server->getGoalCount(), 0);
  EXPECT_EQ(server_handler->backup_server->getGoalCount(), 0);
}

}  // namespace nav2_system_tests

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  return all_successful;
}
