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

#include <chrono>
#include <fstream>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <regex>

#include "gtest/gtest.h"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/create_timer_ros.hpp"

#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

#include "nav2_behavior_tree/plugins_list.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "server_handler.hpp"

using namespace std::chrono_literals;

namespace nav2_system_tests
{

class BehaviorTreeHandler
{
public:
  BehaviorTreeHandler()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("behavior_tree_handler");

    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(), node_->get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    tf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, node_, false);

    odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node_);

    nav2_util::Tokens plugin_libs = nav2_util::split(nav2::details::BT_BUILTIN_PLUGINS, ';');

    for (const auto & p : plugin_libs) {
      factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
    }
  }

  BT::Blackboard::Ptr setBlackboardVariables()
  {
     // Create and populate the blackboard
    blackboard = BT::Blackboard::create();
    blackboard->set("node", node_);
    blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(20));
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout",
             std::chrono::milliseconds(1000));
    blackboard->set("tf_buffer", tf_);
    blackboard->set("initial_pose_received", false);
    blackboard->set("number_recoveries", 0);
    blackboard->set("odom_smoother", odom_smoother_);

    // Create dummy goal
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = "map";
    blackboard->set("goal", goal);
    return blackboard;
  }

  std::optional<std::string> extractBehaviorTreeID(
    const std::string & file_or_id)
  {
    // If it’s not an .xml, treat it as a plain ID
    if (file_or_id.size() < 4 || file_or_id.substr(file_or_id.size() - 4) != ".xml") {
      return file_or_id;
    }

    std::ifstream file(file_or_id);
    if (!file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "Could not open file %s", file_or_id.c_str());
      return std::nullopt;
    }

    const std::regex root_regex(R"(<root[^>]*main_tree_to_execute="(.*?)");
    const std::regex id_regex(R"(<BehaviorTree\s+ID="(.*?)");

    std::string line;
    while (std::getline(file, line)) {
      std::smatch match;
      // First look for <root main_tree_to_execute="...">
      if (std::regex_search(line, match, root_regex) && match.size() > 1) {
        return match[1].str();
      }
      // Otherwise fall back to the first <BehaviorTree ID="...">
      if (std::regex_search(line, match, id_regex) && match.size() > 1) {
        return match[1].str();
      }
    }

    return std::nullopt;
  }

  bool loadBehaviorTree(
    const std::string & file_or_id,
    const std::vector<std::string> & search_directories)
  {
    namespace fs = std::filesystem;
    auto bt_id = extractBehaviorTreeID(file_or_id);
    if (!bt_id) {
      RCLCPP_ERROR(node_->get_logger(),
        "Failed to extract BehaviorTree ID from: %s", file_or_id.c_str());
      return false;
    }

    // Register all XML behavior subtrees in the directories
    for (const auto & directory : search_directories) {
      try {
        for (const auto & entry : fs::directory_iterator(directory)) {
          if (entry.path().extension() == ".xml") {
            factory_.registerBehaviorTreeFromFile(entry.path().string());
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(),
          "Exception reading behavior tree directory: %s", e.what());
        return false;
      }
    }

    // Create and populate the blackboard
    blackboard = setBlackboardVariables();

    // Build the tree from the ID (resolved from <root> or <BehaviorTree ID>)
    try {
      tree = factory_.createTree(*bt_id, blackboard);
    } catch (BT::RuntimeError & exp) {
      RCLCPP_ERROR(node_->get_logger(),
        "Failed to create BT [%s] from %s: %s",
        bt_id->c_str(), file_or_id.c_str(), exp.what());
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
  nav2::LifecycleNode::SharedPtr node_;

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
  // Get the BT root directory
  const auto root_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("nav2_bt_navigator")
    ) / "behavior_trees";

  ASSERT_TRUE(std::filesystem::exists(root_dir));
  ASSERT_TRUE(std::filesystem::is_directory(root_dir));

  std::vector<std::string> search_directories = {root_dir.string()};

  for (auto const & entry : std::filesystem::recursive_directory_iterator(root_dir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".xml") {
      std::string main_bt = entry.path().string();
      std::cout << "Testing BT file: " << main_bt << std::endl;

      EXPECT_TRUE(bt_handler->loadBehaviorTree(main_bt, search_directories))
        << "Failed to load: " << main_bt;
    }
  }
}

TEST_F(BehaviorTreeTestFixture, TestWrongBTFormatXML)
{
  auto write_file = [](const std::string & path, const std::string & content) {
      std::ofstream ofs(path);
      ofs << content;
    };

  // File paths
  std::string valid_subtree = "/tmp/valid_subtree.xml";
  std::string invalid_subtree = "/tmp/invalid_subtree.xml";
  std::string main_file = "/tmp/test_main_tree.xml";
  std::string malformed_main = "/tmp/malformed_main.xml";

  // Valid subtree
  write_file(valid_subtree,
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<root BTCPP_format=\"4\">\n"
    "    <BehaviorTree ID=\"NoopTree\">\n"
    "        <AlwaysSuccess />\n"
    "    </BehaviorTree>\n"
    "</root>\n");

  // Invalid subtree (malformed XML)
  write_file(invalid_subtree, "<root><invalid></root>");

  // Main tree referencing the valid subtree
  write_file(main_file,
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n"
    "  <BehaviorTree ID=\"MainTree\">\n"
    "    <Subtree ID=\"NoopTree\"/>\n"
    "  </BehaviorTree>\n"
    "</root>\n");

  // Malformed main tree
  write_file(malformed_main, "<root><invalid></root>");

  std::vector<std::string> search_directories = {"/tmp"};

  EXPECT_FALSE(bt_handler->loadBehaviorTree(main_file, search_directories));
  EXPECT_FALSE(bt_handler->loadBehaviorTree(malformed_main, search_directories));

  std::remove(valid_subtree.c_str());
  std::remove(main_file.c_str());
  std::remove(invalid_subtree.c_str());
  std::remove(malformed_main.c_str());
}

TEST_F(BehaviorTreeTestFixture, TestExtractBehaviorTreeID)
{
  // Helper for writing files
  auto write_file = [](const std::string & path, const std::string & content) {
      std::ofstream ofs(path);
      ofs << content;
    };

  // 1. Valid XML with ID
  std::string valid_xml = "/tmp/extract_bt_id_valid.xml";
  write_file(valid_xml,
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<root BTCPP_format=\"4\">\n"
    "  <BehaviorTree ID=\"TestTree\">\n"
    "    <AlwaysSuccess />\n"
    "  </BehaviorTree>\n"
    "</root>\n");
  auto id = bt_handler->extractBehaviorTreeID(valid_xml);
  EXPECT_TRUE(id.has_value());
  EXPECT_EQ(*id, "TestTree");

  // 2. Not an XML file, just an ID string
  auto direct_id = bt_handler->extractBehaviorTreeID("SomeTreeID");
  EXPECT_TRUE(direct_id.has_value());
  EXPECT_EQ(*direct_id, "SomeTreeID");

  // 3. Malformed XML (no ID)
  std::string malformed_xml = "/tmp/extract_bt_id_malformed.xml";
  write_file(malformed_xml, "<root><invalid></root>");
  auto missing_id = bt_handler->extractBehaviorTreeID(malformed_xml);
  EXPECT_FALSE(missing_id.has_value());

  // 4. File does not exist
  auto not_found = bt_handler->extractBehaviorTreeID("/tmp/does_not_exist.xml");
  EXPECT_FALSE(not_found.has_value());

  // Cleanup
  std::remove(valid_xml.c_str());
  std::remove(malformed_xml.c_str());
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
  const auto root_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("nav2_bt_navigator")
    ) / "behavior_trees";
  auto bt_file = root_dir / "navigate_to_pose_w_replanning_and_recovery.xml";

  std::vector<std::string> search_directories = {root_dir.string()};

  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string(), search_directories), true);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  while (result == BT::NodeStatus::RUNNING) {
    result = bt_handler->tree.tickOnce();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success since all action servers returned success
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

  // Goal count should be 1 since only one goal is sent to ComputePathToPose and FollowPath servers
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 1);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_msg, "");

  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 1);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_msg, "");

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
  const auto root_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("nav2_bt_navigator")
    ) / "behavior_trees";
  auto bt_file = root_dir / "navigate_to_pose_w_replanning_and_recovery.xml";

  std::vector<std::string> search_directories = {root_dir.string()};

  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string(), search_directories), true);

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
    result = bt_handler->tree.tickOnce();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be failure
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);

  // Goal count should be 2 since only two goals are sent to ComputePathToPose
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 14);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_code, 207);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_msg, "Timeout");

  // Goal count should be 0 since no goal is sent to FollowPath action server
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 0);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_msg, "");

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
  const auto root_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("nav2_bt_navigator")
    ) / "behavior_trees";
  auto bt_file = root_dir / "navigate_to_pose_w_replanning_and_recovery.xml";

  std::vector<std::string> search_directories = {root_dir.string()};

  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string(), search_directories), true);

  // Set ComputePathToPose and FollowPath action servers to fail for the first action
  Ranges failureRange;
  failureRange.emplace_back(Range(0, 1));
  server_handler->compute_path_to_pose_server->setFailureRanges(failureRange);
  server_handler->follow_path_server->setFailureRanges(failureRange);

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  while (result == BT::NodeStatus::RUNNING) {
    result = bt_handler->tree.tickOnce();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

  // Goal count should be 2 since only two goals were sent to ComputePathToPose and FollowPath
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 2);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_msg, "");

  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 2);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_msg, "");

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
  const auto root_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("nav2_bt_navigator")
    ) / "behavior_trees";
  auto bt_file = root_dir / "navigate_to_pose_w_replanning_and_recovery.xml";

  std::vector<std::string> search_directories = {root_dir.string()};

  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string(), search_directories), true);

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
    result = bt_handler->tree.tickOnce();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

  // FollowPath is called 4 times
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 4);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_msg, "");

  // ComputePathToPose is called 3 times
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 3);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_msg, "");

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
  const auto root_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("nav2_bt_navigator")
    ) / "behavior_trees";
  auto bt_file = root_dir / "navigate_to_pose_w_replanning_and_recovery.xml";

  std::vector<std::string> search_directories = {root_dir.string()};

  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string(), search_directories), true);

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
    result = bt_handler->tree.tickOnce();
    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);

  // ComputePathToPose is called 12 times
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 7);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_msg, "");

  // FollowPath is called 4 times
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 14);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_code, 106);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_msg, "No valid control");

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
  const auto root_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("nav2_bt_navigator")
    ) / "behavior_trees";
  auto bt_file = root_dir / "navigate_to_pose_w_replanning_and_recovery.xml";

  std::vector<std::string> search_directories = {root_dir.string()};

  EXPECT_EQ(bt_handler->loadBehaviorTree(bt_file.string(), search_directories), true);

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
    result = bt_handler->tree.tickOnce();

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
      bt_handler->blackboard->set("goal", goal);  // NOLINT
    }

    std::this_thread::sleep_for(10ms);
  }

  // The final result should be success
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);

  // ComputePathToPose is called 4 times
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getGoalCount(), 3);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->compute_path_to_pose_server->getResult()->error_msg, "");

  // FollowPath is called 3 times
  EXPECT_EQ(server_handler->follow_path_server->getGoalCount(), 5);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_code, 0);
  EXPECT_EQ(server_handler->follow_path_server->getResult()->error_msg, "");

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
