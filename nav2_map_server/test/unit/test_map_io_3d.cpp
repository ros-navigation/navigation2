// Copyright (c) 2020 Shivam Pandey
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
#include <experimental/filesystem>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "test_constants/test_constants.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "nav2_map_server/map_3d/map_io_3d.hpp"

#define TEST_DIR TEST_DIRECTORY

using namespace std;  // NOLINT
using namespace nav2_map_server;  // NOLINT
using std::experimental::filesystem::path;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

class MapIO3DTester : public ::testing::Test
{
protected:
  // Fill LoadParameters with standard for testing values
  // Input: pcd_file_name
  // Output: load_parameters
  static void fillLoadParameters(
    const std::string & pcd_file_name,
    map_3d::LoadParameters & load_parameters)
  {
    load_parameters.pcd_file_name = pcd_file_name;


    // Position
    tf2::Vector3 translation = tf2::Vector3(
      tf2Scalar(0),
      tf2Scalar(0),
      tf2Scalar(0));
    // Orientation
    tf2::Quaternion rotation = tf2::Quaternion(
      tf2Scalar(1),
      tf2Scalar(0),
      tf2Scalar(0),
      tf2Scalar(0));
    // Transform
    load_parameters.origin = tf2::Transform(rotation, translation);
  }

  // Fill SaveParameters with standard for testing values
  // Input: map_file_name, format
  // Output: save_parameters
  static void fillSaveParameters(
    const std::string & map_file_name,
    const std::string & format,
    bool as_binary,
    map_3d::SaveParameters & save_parameters)
  {
    save_parameters.map_file_name = map_file_name;

    save_parameters.as_binary = as_binary;
    save_parameters.format = format;
  }

  static void verifyMapMsg(
    const sensor_msgs::msg::PointCloud2 & map_msg)
  {
    ASSERT_EQ(map_msg.width, g_valid_pcd_width);
    ASSERT_EQ(map_msg.data.size(), g_valid_pcd_data_size);
  }
};

// Load a valid reference PCD file. Check obtained PointCloud2 message for consistency:
// loaded pcd should match the known dimensions and content of the file.
// Save obtained PointCloud2 message into a tmp PCD file. Then load back saved tmp file
// and check for consistency.
// Succeeds all steps were passed without a problem or expection.
TEST_F(MapIO3DTester, loadSaveValidPCD)
{
  // 1. Load reference map file and verify obtained OccupancyGrid
  map_3d::LoadParameters loadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_pcd_file), loadParameters);

  sensor_msgs::msg::PointCloud2 map_msg;
  ASSERT_NO_THROW(map_3d::loadMapFromFile(loadParameters, map_msg));

  // 2. Save OccupancyGrid into a tmp file
  map_3d::SaveParameters saveParameters;

  fillSaveParameters(
    path(g_tmp_dir) / path(g_valid_pcd_map_name),
    "pcd", false, saveParameters);

  ASSERT_TRUE(map_3d::saveMapToFile(map_msg, saveParameters));

  // 3. Load saved map and verify it
  map_3d::LOAD_MAP_STATUS status =
    map_3d::loadMapFromYaml(path(g_tmp_dir) / path(g_valid_pcd_yaml_file), map_msg);
  ASSERT_EQ(status, map_3d::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}

// Load valid YAML file and check for consistency
TEST_F(MapIO3DTester, loadValidYAML)
{
  map_3d::LoadParameters loadParameters;
  ASSERT_NO_THROW(
    loadParameters =
    map_3d::loadMapYaml(path(TEST_DIR) / path(g_valid_pcd_yaml_file)));

  map_3d::LoadParameters refLoadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_pcd_file), refLoadParameters);

  std::experimental::filesystem::path ldParam(loadParameters.pcd_file_name);
  std::experimental::filesystem::path refParam(refLoadParameters.pcd_file_name);

  ASSERT_EQ(ldParam.stem(), refParam.stem());
}

TEST_F(MapIO3DTester, loadInvalidYAML)
{
  map_3d::LoadParameters loadParameters;
  ASSERT_ANY_THROW(
    loadParameters =
    map_3d::loadMapYaml(path(TEST_DIR) / path("invalid_file.yaml")));
}
