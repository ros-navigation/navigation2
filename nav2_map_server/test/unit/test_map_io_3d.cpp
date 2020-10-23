// Copyright (c) 2020 Shivam Pandey pandeyshivam2017robotics@gmail.com
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
//
// Created by shivam on 7/28/20.
//

#include <gtest/gtest.h>
#include <experimental/filesystem>
#include <string>
#include <vector>

#include "nav2_map_server/map_3d/map_io_3d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "test_constants/test_constants.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"

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
  void fillLoadParameters(
    const std::string & pcd_file_name,
    map_3d::LoadParameters & load_parameters)
  {
    load_parameters.pcd_file_name = pcd_file_name;
    load_parameters.origin.center = g_valid_center_pcd;
    load_parameters.origin.orientation = g_valid_orientation_pcd;
  }

  // Fill SaveParameters with standard for testing values
  // Input: map_file_name, format
  // Output: save_parameters
  void fillSaveParameters(
    const std::string & map_file_name,
    const std::vector<float> & center,
    const std::vector<float> & orientation,
    const std::string & format,
    bool as_binary,
    map_3d::SaveParameters & save_parameters)
  {
    save_parameters.origin.resize();
    save_parameters.map_file_name = map_file_name;

    save_parameters.origin.center[0] = center[0];
    save_parameters.origin.center[1] = center[1];
    save_parameters.origin.center[2] = center[2];

    save_parameters.origin.orientation[0] = orientation[0];
    save_parameters.origin.orientation[1] = orientation[1];
    save_parameters.origin.orientation[2] = orientation[2];
    save_parameters.origin.orientation[3] = orientation[3];

    save_parameters.as_binary = as_binary;
    save_parameters.format = format;
  }

  static void verifyMapMsg(
    const sensor_msgs::msg::PointCloud2 & map_msg,
    const geometry_msgs::msg::Pose & origin)
  {
    std::vector<float> center;
    center.push_back(origin.position.x);
    center.push_back(origin.position.y);
    center.push_back(origin.position.z);

    std::vector<float> orientation;
    orientation.push_back(origin.orientation.w);
    orientation.push_back(origin.orientation.x);
    orientation.push_back(origin.orientation.y);
    orientation.push_back(origin.orientation.z);
    ASSERT_EQ(center, g_valid_center_pcd);
    ASSERT_EQ(orientation, g_valid_orientation_pcd);

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
  std::cout << "[debug] " << "filled load parameters" << std::endl;

  sensor_msgs::msg::PointCloud2 map_msg;
  geometry_msgs::msg::Pose origin;
  ASSERT_NO_THROW(map_3d::loadMapFromFile(loadParameters, map_msg, origin));

  std::cout << "[debug]" << "file_loaded" << std::endl;

  // 2. Save OccupancyGrid into a tmp file
  map_3d::SaveParameters saveParameters;
  std::vector<float> center(3);
  std::vector<float> orientation(4);

  // Set view_point translation(origin)
  center[0] = origin.position.x;
  center[1] = origin.position.y;
  center[2] = origin.position.z;

  // Set view_point orientation
  orientation[0] = origin.orientation.w;
  orientation[1] = origin.orientation.x;
  orientation[2] = origin.orientation.y;
  orientation[3] = origin.orientation.z;

  std::cout << "[debug]" << "origin loaded" << std::endl;

  ASSERT_EQ(center, g_valid_center_pcd);
  ASSERT_EQ(orientation, g_valid_orientation_pcd);

  fillSaveParameters(
    path(g_tmp_dir) / path(g_valid_pcd_map_name),
    center, orientation, "pcd", false, saveParameters);

  std::cout << "[debug] " << "filled save parameters" << std::endl;

  ASSERT_TRUE(map_3d::saveMapToFile(map_msg, saveParameters));

  std::cout << "[debug] " << "save file is done" << std::endl;

  // 3. Load saved map and verify it
  map_3d::LOAD_MAP_STATUS status =
    map_3d::loadMapFromYaml(path(g_tmp_dir) / path(g_valid_pcd_yaml_file), map_msg, origin);
  ASSERT_EQ(status, map_3d::LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg, origin);
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
