//
// Created by shivam on 7/28/20.
//

#include <gtest/gtest.h>
#include <experimental/filesystem>
#include <string>
#include <vector>

#include "nav2_map_server_3D/map_io_3D.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "test_constants/test_constants.h"

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
      nav2_map_server_3D::LoadParameters_3D & load_parameters)
  {
    load_parameters.pcd_file_name =pcd_file_name;
    load_parameters.view_point = g_valid_view_point;
  }

  // Fill SaveParameters with standard for testing values
  // Input: map_file_name, format
  // Output: save_parameters
  void fillSaveParameters(
      const std::string & map_file_name,
      const std::vector<float>& view_point,
      const std::string & format,
      bool as_binary,
      nav2_map_server_3D::SaveParameters & save_parameters)
  {
    save_parameters.map_file_name = map_file_name;
    save_parameters.view_point = view_point;
    save_parameters.as_binary = as_binary;
    save_parameters.format = format;
  }

  // TODO: verify PCD data
};

// Load a valid reference PCD file. Check obtained PointCloud2 message for consistency:
// loaded pcd should match the known dimensions and content of the file.
// Save obtained PointCloud2 message into a tmp PCD file. Then load back saved tmp file
// and check for consistency.
// Succeeds all steps were passed without a problem or expection.
TEST_F(MapIO3DTester, loadSaveValidPCD)
{
  // 1. Load reference map file and verify obtained OccupancyGrid
  nav2_map_server_3D::LoadParameters_3D loadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_pcd_file), loadParameters);

  sensor_msgs::msg::PointCloud2 map_msg;
  geometry_msgs::msg::Transform view_point_msg;
  ASSERT_NO_THROW(nav2_map_server_3D::loadMapFromFile(loadParameters, map_msg, view_point_msg));

  //  verifyMapMsg(map_msg); TODO: varification for pcd file

  // 2. Save OccupancyGrid into a tmp file
  nav2_map_server_3D::SaveParameters saveParameters;
  std::vector<float> view_point(7);
  // Set view_point translation(origin)
  view_point[0] = view_point_msg.translation.x;
  view_point[1] = view_point_msg.translation.y;
  view_point[2] = view_point_msg.translation.z;

  // Set view_point orientation
  view_point[3] = view_point_msg.rotation.w;
  view_point[4] = view_point_msg.rotation.x;
  view_point[5] = view_point_msg.rotation.y;
  view_point[6] = view_point_msg.rotation.z;

  ASSERT_EQ(view_point, g_valid_view_point);

  fillSaveParameters(path(g_tmp_dir) / path(g_valid_pcd_map_name), view_point, "pcd", false,saveParameters);

  ASSERT_TRUE(nav2_map_server_3D::saveMapToFile(map_msg, saveParameters));

  // 3. Load saved map and verify it
  nav2_map_server_3D::LOAD_MAP_STATUS status = nav2_map_server_3D::loadMapFromYaml(path(g_tmp_dir) / path
      (g_valid_pcd_yaml_file), map_msg, view_point_msg);
  ASSERT_EQ(status, nav2_map_server_3D::LOAD_MAP_SUCCESS);

  //  verifyMapMsg(map_msg); TODO: verify msg data
}

// Load valid YAML file and check for consistency
TEST_F(MapIO3DTester, loadValidYAML)
{
  nav2_map_server_3D::LoadParameters_3D loadParameters;
  ASSERT_NO_THROW(loadParameters = nav2_map_server_3D::loadMapYaml(path(TEST_DIR) / path(g_valid_pcd_yaml_file)));

  nav2_map_server_3D::LoadParameters_3D refLoadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_pcd_file), refLoadParameters);
  ASSERT_EQ(loadParameters.pcd_file_name, refLoadParameters.pcd_file_name);
}

TEST_F(MapIO3DTester, loadInvalidYAML)
{
  nav2_map_server_3D::LoadParameters_3D loadParameters;
  ASSERT_ANY_THROW(loadParameters = nav2_map_server_3D::loadMapYaml(path(TEST_DIR) / path("invalid_file.yaml")));
}