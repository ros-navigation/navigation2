//
// Created by shivam on 7/28/20.
//

#include <gtest/gtest.h>
#include <experimental/filesystem>
#include <string>
#include <vector>

#include "nav2_map_server_3d/map_io_3d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "test_constants/test_constants.h"
#include "nav2_msgs/msg/pcd2.hpp"
#include "boost/filesystem.hpp"

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
    nav2_map_server_3d::LoadParameters & load_parameters)
  {
    load_parameters.pcd_file_name =pcd_file_name;
    load_parameters.origin = g_valid_origin_pcd;
    load_parameters.orientation = g_valid_orientation_pcd;
  }

  // Fill SaveParameters with standard for testing values
  // Input: map_file_name, format
  // Output: save_parameters
  void fillSaveParameters(
    const std::string & map_file_name,
    const std::vector<float> & origin,
    const std::vector<float> & orientation,
    const std::string & format,
    bool as_binary,
    nav2_map_server_3d::SaveParameters & save_parameters)
  {
    save_parameters.map_file_name = map_file_name;
    save_parameters.origin = origin;
    save_parameters.orientation = orientation;
    save_parameters.as_binary = as_binary;
    save_parameters.format = format;
  }

  static void verifyMapMsg(const nav2_msgs::msg::PCD2 & map_msg)
  {
    std::vector<float> origin;
    origin.push_back(map_msg.origin.x);
    origin.push_back(map_msg.origin.y);
    origin.push_back(map_msg.origin.z);

    std::vector<float> orientation;
    orientation.push_back(map_msg.orientation.w);
    orientation.push_back(map_msg.orientation.x);
    orientation.push_back(map_msg.orientation.y);
    orientation.push_back(map_msg.orientation.z);
    ASSERT_EQ(origin, g_valid_origin_pcd);
    ASSERT_EQ(orientation, g_valid_orientation_pcd);

    ASSERT_EQ(map_msg.map.width, g_valid_pcd_width);
    ASSERT_EQ(map_msg.map.data.size(), g_valid_pcd_data_size);
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
  nav2_map_server_3d::LoadParameters loadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_pcd_file), loadParameters);

  nav2_msgs::msg::PCD2 map_msg;
  ASSERT_NO_THROW(nav2_map_server_3d::loadMapFromFile(loadParameters, map_msg));

  // 2. Save OccupancyGrid into a tmp file
  nav2_map_server_3d::SaveParameters saveParameters;
  std::vector<float> origin(3);
  std::vector<float> orientation(4);
  // Set view_point translation(origin)
  origin[0] = map_msg.origin.x;
  origin[1] = map_msg.origin.y;
  origin[2] = map_msg.origin.z;

  // Set view_point orientation
  orientation[0] = map_msg.orientation.w;
  orientation[1] = map_msg.orientation.x;
  orientation[2] = map_msg.orientation.y;
  orientation[3] = map_msg.orientation.z;

  ASSERT_EQ(origin, g_valid_origin_pcd);
  ASSERT_EQ(orientation, g_valid_orientation_pcd);

  fillSaveParameters(path(g_tmp_dir) / path(g_valid_pcd_map_name),
                     origin, orientation, "pcd", false,saveParameters);

  ASSERT_TRUE(nav2_map_server_3d::saveMapToFile(map_msg.map, saveParameters));

  // 3. Load saved map and verify it
  nav2_map_server_3d::LOAD_MAP_STATUS status =
    nav2_map_server_3d::loadMapFromYaml(path(g_tmp_dir) / path(g_valid_pcd_yaml_file), map_msg);
  ASSERT_EQ(status, nav2_map_server_3d::LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}

// Load valid YAML file and check for consistency
TEST_F(MapIO3DTester, loadValidYAML)
{
  nav2_map_server_3d::LoadParameters loadParameters;
  ASSERT_NO_THROW(loadParameters = nav2_map_server_3d::loadMapYaml(path(TEST_DIR) / path(g_valid_pcd_yaml_file)));

  nav2_map_server_3d::LoadParameters refLoadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_pcd_file), refLoadParameters);

  boost::filesystem::path ldParam(loadParameters.pcd_file_name);
  boost::filesystem::path refParam(refLoadParameters.pcd_file_name);

  ASSERT_EQ(ldParam.stem(), refParam.stem());
}

TEST_F(MapIO3DTester, loadInvalidYAML)
{
  nav2_map_server_3d::LoadParameters loadParameters;
  ASSERT_ANY_THROW(loadParameters = nav2_map_server_3d::loadMapYaml(path(TEST_DIR) / path("invalid_file.yaml")));
}