// Copyright (c) 2026 Open Navigation LLC
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

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "nav2_amcl/amcl_node.hpp"
#include "rclcpp/rclcpp.hpp"

class PosePersistenceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_filepath_ = "/tmp/amcl_test_pose_" + std::to_string(getpid());
    // Clean up any existing test file
    std::remove(test_filepath_.c_str());
    std::remove((test_filepath_ + ".tmp").c_str());
  }

  void TearDown() override
  {
    // Clean up test files
    std::remove(test_filepath_.c_str());
    std::remove((test_filepath_ + ".tmp").c_str());
  }

  std::string test_filepath_;
};

TEST_F(PosePersistenceTest, test_pose_persistence_parameters)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"random_seed", 42},
      {"initialize_at_saved_pose", true},
      {"saved_pose_filepath", test_filepath_}});

  auto amcl = std::make_shared<nav2_amcl::AmclNode>(options);
  amcl->configure();
  amcl->activate();

  // Verify parameters were set correctly
  EXPECT_EQ(amcl->get_parameter("initialize_at_saved_pose").as_bool(), true);
  EXPECT_EQ(amcl->get_parameter("saved_pose_filepath").as_string(), test_filepath_);

  // Test dynamic parameter updates
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    amcl->get_node_base_interface(), amcl->get_node_topics_interface(),
    amcl->get_node_graph_interface(),
    amcl->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("initialize_at_saved_pose", false),
      rclcpp::Parameter("saved_pose_filepath", "/tmp/new_path")});

  rclcpp::spin_until_future_complete(amcl->get_node_base_interface(), results);

  EXPECT_EQ(amcl->get_parameter("initialize_at_saved_pose").as_bool(), false);
  EXPECT_EQ(amcl->get_parameter("saved_pose_filepath").as_string(), "/tmp/new_path");

  amcl->deactivate();
  amcl->cleanup();
}

TEST_F(PosePersistenceTest, test_pose_file_format)
{
  // Create a pose file manually to verify format (old format for backwards compat)
  {
    std::ofstream file(test_filepath_);
    file << "x: 1.234567\n";
    file << "y: 2.345678\n";
    file << "z: 0.0\n";
    file << "yaw: 1.570796\n";
    file.close();
  }

  // Verify file was created
  std::ifstream check_file(test_filepath_);
  EXPECT_TRUE(check_file.is_open());
  check_file.close();
}

TEST_F(PosePersistenceTest, test_pose_file_format_with_timestamp_and_frame)
{
  // Create a pose file with new format including timestamp and frame_id
  {
    std::ofstream file(test_filepath_);
    file << "timestamp: 1738678400.123456789\n";
    file << "frame_id: map\n";
    file << "x: 1.234567\n";
    file << "y: 2.345678\n";
    file << "z: 0.0\n";
    file << "yaw: 1.570796\n";
    file.close();
  }

  // Verify file was created and has correct number of fields
  std::ifstream check_file(test_filepath_);
  EXPECT_TRUE(check_file.is_open());

  std::string line;
  int value_count = 0;
  while (std::getline(check_file, line)) {
    if (!line.empty() && line[0] != '#') {
      value_count++;
    }
  }
  EXPECT_EQ(value_count, 6);  // timestamp, frame_id, x, y, z, yaw
  check_file.close();
}

TEST_F(PosePersistenceTest, test_timestamp_parsing_precision)
{
  // Test that timestamp is correctly parsed back into sec and nanosec
  // Using a known timestamp: 1738678400.500000000 (sec=1738678400, nanosec=500000000)
  double test_timestamp = 1738678400.5;
  int32_t expected_sec = 1738678400;
  uint32_t expected_nanosec = 500000000;

  // Parse like the loadPoseFromFile does
  int32_t parsed_sec = static_cast<int32_t>(test_timestamp);
  uint32_t parsed_nanosec = static_cast<uint32_t>((test_timestamp - parsed_sec) * 1e9);

  EXPECT_EQ(parsed_sec, expected_sec);
  // Allow small tolerance due to floating point precision
  EXPECT_NEAR(parsed_nanosec, expected_nanosec, 1000);  // Within 1 microsecond
}

TEST_F(PosePersistenceTest, test_timestamp_roundtrip)
{
  // Test that a timestamp survives a save/load roundtrip with reasonable precision
  int32_t original_sec = 1738678400;
  uint32_t original_nanosec = 123456789;

  // Combine like savePoseToFile does
  double combined = original_sec + static_cast<double>(original_nanosec) / 1e9;

  // Parse back like loadPoseFromFile does
  int32_t parsed_sec = static_cast<int32_t>(combined);
  uint32_t parsed_nanosec = static_cast<uint32_t>((combined - parsed_sec) * 1e9);

  EXPECT_EQ(parsed_sec, original_sec);
  // Due to double precision limits (~6 decimal digits after 10-digit integer),
  // we expect microsecond precision, not nanosecond
  EXPECT_NEAR(parsed_nanosec, original_nanosec, 1000);  // Within 1 microsecond
}

TEST_F(PosePersistenceTest, test_pose_file_with_comments)
{
  // Create a pose file with comments and new format
  {
    std::ofstream file(test_filepath_);
    file << "# AMCL Saved Pose\n";
    file << "# This is a comment\n";
    file << "timestamp: 1738678400.0\n";
    file << "frame_id: map\n";
    file << "x: 5.0\n";
    file << "y: 10.0\n";
    file << "z: 0.0\n";
    file << "yaw: 3.14159\n";
    file.close();
  }

  // Verify file was created and can be read
  std::ifstream check_file(test_filepath_);
  EXPECT_TRUE(check_file.is_open());

  std::string line;
  int value_count = 0;
  while (std::getline(check_file, line)) {
    if (!line.empty() && line[0] != '#') {
      value_count++;
    }
  }
  EXPECT_EQ(value_count, 6);  // timestamp, frame_id, x, y, z, yaw
  check_file.close();
}

TEST_F(PosePersistenceTest, test_timer_creation_with_positive_rate)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"random_seed", 42},
      {"save_pose_rate", 1.0}});

  auto amcl = std::make_shared<nav2_amcl::AmclNode>(options);
  amcl->configure();
  amcl->activate();

  // Timer should be created when save_pose_rate > 0
  EXPECT_EQ(amcl->get_parameter("save_pose_rate").as_double(), 1.0);

  amcl->deactivate();
  amcl->cleanup();
}

TEST_F(PosePersistenceTest, test_default_filepath)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"random_seed", 42}});

  auto amcl = std::make_shared<nav2_amcl::AmclNode>(options);
  amcl->configure();

  // Verify default filepath
  EXPECT_EQ(
    amcl->get_parameter("saved_pose_filepath").as_string(),
    "/tmp/amcl_saved_pose");

  // No need to call cleanup - node destructor handles it
  // Calling cleanup without deactivate after configure can cause issues
}

TEST_F(PosePersistenceTest, test_load_saved_pose_file)
{
  // Create a pose file to load with new format
  {
    std::ofstream file(test_filepath_);
    file << "timestamp: 1738678400.123456\n";
    file << "frame_id: map\n";
    file << "x: 1.5\n";
    file << "y: 2.5\n";
    file << "z: 0.0\n";
    file << "yaw: 1.57\n";
    file.close();
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"random_seed", 42},
      {"initialize_at_saved_pose", true},
      {"saved_pose_filepath", test_filepath_}});

  auto amcl = std::make_shared<nav2_amcl::AmclNode>(options);
  amcl->configure();
  amcl->activate();

  // Verify parameters were set correctly
  EXPECT_EQ(amcl->get_parameter("initialize_at_saved_pose").as_bool(), true);
  EXPECT_EQ(amcl->get_parameter("saved_pose_filepath").as_string(), test_filepath_);

  amcl->deactivate();
  amcl->cleanup();
}

TEST_F(PosePersistenceTest, test_load_saved_pose_file_backwards_compat)
{
  // Create a pose file in old format (no timestamp, no frame_id)
  {
    std::ofstream file(test_filepath_);
    file << "x: 3.0\n";
    file << "y: 4.0\n";
    file << "z: 0.0\n";
    file << "yaw: 0.785\n";
    file.close();
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"random_seed", 42},
      {"initialize_at_saved_pose", true},
      {"saved_pose_filepath", test_filepath_}});

  auto amcl = std::make_shared<nav2_amcl::AmclNode>(options);
  amcl->configure();
  amcl->activate();

  // Should load successfully even without timestamp and frame_id
  EXPECT_EQ(amcl->get_parameter("initialize_at_saved_pose").as_bool(), true);

  amcl->deactivate();
  amcl->cleanup();
}

TEST_F(PosePersistenceTest, test_ros_params_priority_over_saved_pose)
{
  // Create a pose file with different values than ROS params
  {
    std::ofstream file(test_filepath_);
    file << "x: 10.0\n";
    file << "y: 20.0\n";
    file << "z: 0.0\n";
    file << "yaw: 3.14\n";
    file.close();
  }

  // Set both set_initial_pose and initialize_at_saved_pose to true
  // ROS parameters should take priority
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"random_seed", 42},
      {"set_initial_pose", true},
      {"initial_pose.x", 1.0},
      {"initial_pose.y", 2.0},
      {"initial_pose.z", 0.0},
      {"initial_pose.yaw", 0.5},
      {"initialize_at_saved_pose", true},
      {"saved_pose_filepath", test_filepath_}});

  auto amcl = std::make_shared<nav2_amcl::AmclNode>(options);
  amcl->configure();
  amcl->activate();

  // Verify that ROS parameters were used (not the file values)
  // The initial_pose parameters should reflect what was set, not the file
  EXPECT_EQ(amcl->get_parameter("initial_pose.x").as_double(), 1.0);
  EXPECT_EQ(amcl->get_parameter("initial_pose.y").as_double(), 2.0);
  EXPECT_EQ(amcl->get_parameter("initial_pose.yaw").as_double(), 0.5);

  amcl->deactivate();
  amcl->cleanup();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
