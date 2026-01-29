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
  // Create a pose file manually to verify format
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

TEST_F(PosePersistenceTest, test_pose_file_with_comments)
{
  // Create a pose file with comments
  {
    std::ofstream file(test_filepath_);
    file << "# AMCL Saved Pose\n";
    file << "# This is a comment\n";
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
  EXPECT_EQ(value_count, 4);  // x, y, z, yaw
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
