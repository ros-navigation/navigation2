// Copyright 2019 Rover Robotics

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#include <gtest/gtest.h>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <cstdint>

#include "yaml-cpp/yaml.h"
#include "nav2_map_server/map_io.hpp"
#include "nav2_map_server/map_server.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "test_constants/test_constants.h"

#define TEST_DIR TEST_DIRECTORY

using namespace std;  // NOLINT
using namespace nav2_map_server;  // NOLINT
using std::filesystem::path;

class MapIOTester : public ::testing::Test
{
protected:
  // Fill LoadParameters with standard for testing values
  // Input: image_file_name
  // Output: load_parameters
  void fillLoadParameters(
    const std::string & image_file_name,
    LoadParameters & load_parameters)
  {
    load_parameters.image_file_name = image_file_name;
    load_parameters.resolution = g_valid_image_res;
    load_parameters.origin = g_valid_origin;
    load_parameters.free_thresh = g_default_free_thresh;
    load_parameters.occupied_thresh = g_default_occupied_thresh;
    load_parameters.mode = MapMode::Trinary;
    load_parameters.negate = 0;
  }

  // Fill SaveParameters with standard for testing values
  // Input: map_file_name, image_format
  // Output: save_parameters
  void fillSaveParameters(
    const std::string & map_file_name,
    const std::string & image_format,
    SaveParameters & save_parameters)
  {
    save_parameters.map_file_name = map_file_name;
    save_parameters.image_format = image_format;
    save_parameters.free_thresh = g_default_free_thresh;
    save_parameters.occupied_thresh = g_default_occupied_thresh;
    save_parameters.mode = MapMode::Trinary;
  }

  // Write a single-row binary PGM holding the given gray levels
  // Input: file_name, grays
  // Output: full path of the written file
  std::string writeGrayPgm(const std::string & file_name, const std::vector<uint8_t> & grays)
  {
    const std::string file_path = path(g_tmp_dir) / path(file_name);
    std::ofstream out(file_path, std::ios::binary);
    out << "P5\n" << grays.size() << " 1\n255\n";
    out.write(reinterpret_cast<const char *>(grays.data()), grays.size());
    out.close();
    return file_path;
  }

  // Build a map holding every gray level once, in ascending order
  std::vector<uint8_t> grayGradient()
  {
    std::vector<uint8_t> grays(256);
    for (unsigned int i = 0; i < grays.size(); i++) {
      grays[i] = static_cast<uint8_t>(i);
    }
    return grays;
  }

  // Check that map_msg corresponds to reference pattern
  // Input: map_msg
  void verifyMapMsg(const nav_msgs::msg::OccupancyGrid & map_msg)
  {
    ASSERT_FLOAT_EQ(map_msg.info.resolution, g_valid_image_res);
    ASSERT_EQ(map_msg.info.width, g_valid_image_width);
    ASSERT_EQ(map_msg.info.height, g_valid_image_height);
    for (unsigned int i = 0; i < map_msg.info.width * map_msg.info.height; i++) {
      ASSERT_EQ(g_valid_image_content[i], map_msg.data[i]);
    }
  }
};

// Load a valid reference PGM file. Check obtained OccupancyGrid message for consistency:
// loaded image should match the known dimensions and content of the file.
// Save obtained OccupancyGrid message into a tmp PGM file. Then load back saved tmp file
// and check for consistency.
// Succeeds all steps were passed without a problem or exception.
TEST_F(MapIOTester, loadSaveValidPGM)
{
  // 1. Load reference map file and verify obtained OccupancyGrid
  LoadParameters loadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_pgm_file), loadParameters);

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  verifyMapMsg(map_msg);

  // 2. Save OccupancyGrid into a tmp file
  SaveParameters saveParameters;
  fillSaveParameters(path(g_tmp_dir) / path(g_valid_map_name), "pgm", saveParameters);

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));

  // 3. Load saved map and verify it
  LOAD_MAP_STATUS status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}

// Load a valid reference PNG file. Check obtained OccupancyGrid message for consistency:
// loaded image should match the known dimensions and content of the file.
// Save obtained OccupancyGrid message into a tmp PNG file. Then load back saved tmp file
// and check for consistency.
// Succeeds all steps were passed without a problem or exception.
TEST_F(MapIOTester, loadSaveValidPNG)
{
  // 1. Load reference map file and verify obtained OccupancyGrid
  LoadParameters loadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_png_file), loadParameters);

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  verifyMapMsg(map_msg);

  // 2. Save OccupancyGrid into a tmp file
  SaveParameters saveParameters;
  fillSaveParameters(path(g_tmp_dir) / path(g_valid_map_name), "png", saveParameters);

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));

  // 3. Load saved map and verify it
  LOAD_MAP_STATUS status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}

// Load a valid reference BMP file. Check obtained OccupancyGrid message for consistency:
// loaded image should match the known dimensions and content of the file.
// Save obtained OccupancyGrid message into a tmp BMP file. Then load back saved tmp file
// and check for consistency.
// Succeeds all steps were passed without a problem or exception.
TEST_F(MapIOTester, loadSaveValidBMP)
{
  // 1. Load reference map file and verify obtained OccupancyGrid
  auto test_bmp = path(TEST_DIR) / path(g_valid_bmp_file);

  LoadParameters loadParameters;
  fillLoadParameters(test_bmp, loadParameters);

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  verifyMapMsg(map_msg);

  // 2. Save OccupancyGrid into a tmp file
  SaveParameters saveParameters;
  fillSaveParameters(path(g_tmp_dir) / path(g_valid_map_name), "bmp", saveParameters);

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));

  // 3. Load saved map and verify it
  LOAD_MAP_STATUS status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}

// Load map from a valid file. Trying to save map with different modes.
// Succeeds all steps were passed without a problem or exception.
TEST_F(MapIOTester, loadSaveMapModes)
{
  // 1. Load map from YAML file
  nav_msgs::msg::OccupancyGrid map_msg;
  LOAD_MAP_STATUS status = loadMapFromYaml(path(TEST_DIR) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  // No need to check Trinary mode. This already verified in previous testcases.
  // 2. Save map in Scale mode.
  SaveParameters saveParameters;
  fillSaveParameters(path(g_tmp_dir) / path(g_valid_map_name), "png", saveParameters);
  saveParameters.mode = MapMode::Scale;

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));

  // 3. Load saved map and verify it
  status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);

  // 4. Save map in Raw mode.
  saveParameters.mode = MapMode::Raw;

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));

  // 5. Load saved map and verify it
  status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}

// Load a map holding every gray level in Scale mode. Gray levels between the two
// thresholds must be interpolated onto [0, 100] rather than collapsed onto the
// extremes, and the thresholds themselves must land on an exact gray level.
TEST_F(MapIOTester, loadScaleModeInterpolatesInBetweenValues)
{
  const std::vector<uint8_t> grays = grayGradient();
  const std::string gradient_file = writeGrayPgm("gradient_map.pgm", grays);

  // 1. Load the gradient in Scale mode
  LoadParameters loadParameters;
  fillLoadParameters(gradient_file, loadParameters);
  loadParameters.mode = MapMode::Scale;

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));
  ASSERT_EQ(static_cast<size_t>(map_msg.info.width), grays.size());
  ASSERT_EQ(static_cast<size_t>(map_msg.info.height), 1u);

  // 2. Both extremes, the gray level each threshold falls on, and the midpoint
  EXPECT_EQ(static_cast<int>(map_msg.data[0]), 100);
  EXPECT_EQ(static_cast<int>(map_msg.data[89]), 100);
  EXPECT_EQ(static_cast<int>(map_msg.data[90]), 99);
  EXPECT_EQ(static_cast<int>(map_msg.data[147]), 50);
  EXPECT_EQ(static_cast<int>(map_msg.data[204]), 1);
  EXPECT_EQ(static_cast<int>(map_msg.data[205]), 0);
  EXPECT_EQ(static_cast<int>(map_msg.data[255]), 0);

  // 3. The scaled values decrease monotonically and stay inside [0, 100]
  unsigned int in_between_count = 0;
  for (unsigned int i = 0; i < grays.size(); i++) {
    EXPECT_GE(static_cast<int>(map_msg.data[i]), 0) << "at gray level " << i;
    EXPECT_LE(static_cast<int>(map_msg.data[i]), 100) << "at gray level " << i;
    if (i > 0) {
      EXPECT_LE(map_msg.data[i], map_msg.data[i - 1]) << "at gray level " << i;
    }
    if (map_msg.data[i] > 0 && map_msg.data[i] < 100) {
      in_between_count++;
    }
  }
  EXPECT_EQ(in_between_count, 115u);

  // 4. Negating the image classifies the same gradient from the opposite end
  loadParameters.negate = true;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  EXPECT_EQ(static_cast<int>(map_msg.data[0]), 0);
  EXPECT_EQ(static_cast<int>(map_msg.data[108]), 50);
  EXPECT_EQ(static_cast<int>(map_msg.data[165]), 99);
  EXPECT_EQ(static_cast<int>(map_msg.data[166]), 100);
  EXPECT_EQ(static_cast<int>(map_msg.data[255]), 100);
}

// Trinary mode shares the classification of Scale mode but has no in-between band:
// every gray level between the thresholds must come back as UNKNOWN.
TEST_F(MapIOTester, loadTrinaryModeMarksInBetweenUnknown)
{
  const std::string gradient_file = writeGrayPgm("gradient_map.pgm", grayGradient());

  LoadParameters loadParameters;
  fillLoadParameters(gradient_file, loadParameters);

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  EXPECT_EQ(static_cast<int>(map_msg.data[89]), 100);
  EXPECT_EQ(static_cast<int>(map_msg.data[90]), -1);
  EXPECT_EQ(static_cast<int>(map_msg.data[147]), -1);
  EXPECT_EQ(static_cast<int>(map_msg.data[205]), -1);
  EXPECT_EQ(static_cast<int>(map_msg.data[206]), 0);
}

// The thresholds are compared in float against (1 - gray/255). Rewriting that
// comparison to work on raw gray values instead is equivalent in real arithmetic
// but rounds differently, which flips the classification of the pixel sitting on
// the threshold. Pin that pixel so such a rewrite cannot slip through.
TEST_F(MapIOTester, loadClassifiesThresholdBoundaryExactly)
{
  const std::string boundary_file = writeGrayPgm("boundary_map.pgm", {126, 127});

  LoadParameters loadParameters;
  fillLoadParameters(boundary_file, loadParameters);
  loadParameters.occupied_thresh = 129.0 / 255.0;

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  EXPECT_EQ(static_cast<int>(map_msg.data[0]), 100);
  EXPECT_EQ(static_cast<int>(map_msg.data[1]), -1);
}

// Try to load an invalid file with different ways.
// Succeeds if all cases are got expected fail behaviours.
TEST_F(MapIOTester, loadInvalidFile)
{
  // 1. Trying to load incorrect map by loadMapFromFile()
  auto test_invalid = path(TEST_DIR) / path("foo");

  LoadParameters loadParameters;
  fillLoadParameters(test_invalid, loadParameters);

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_ANY_THROW(loadMapFromFile(loadParameters, map_msg));

  // 2. Trying to load incorrect map by loadMapFromYaml()
  LOAD_MAP_STATUS status = loadMapFromYaml("", map_msg);
  ASSERT_EQ(status, MAP_DOES_NOT_EXIST);

  status = loadMapFromYaml(std::string(test_invalid) + ".yaml", map_msg);
  ASSERT_EQ(status, INVALID_MAP_METADATA);
}

// Load map from a valid file. Trying to save map with different sets of parameters.
// Succeeds if all cases got expected behaviours.
TEST_F(MapIOTester, saveInvalidParameters)
{
  // 1. Load map from YAML file
  nav_msgs::msg::OccupancyGrid map_msg;
  LOAD_MAP_STATUS status = loadMapFromYaml(path(TEST_DIR) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  // 2. Trying to save map with different sets of parameters
  SaveParameters saveParameters;

  saveParameters.map_file_name = path(g_tmp_dir) / path(g_valid_map_name);
  saveParameters.image_format = "";
  saveParameters.free_thresh = 2.0;
  saveParameters.occupied_thresh = 2.0;
  saveParameters.mode = MapMode::Trinary;
  ASSERT_FALSE(saveMapToFile(map_msg, saveParameters));

  saveParameters.free_thresh = -2.0;
  saveParameters.occupied_thresh = -2.0;
  ASSERT_FALSE(saveMapToFile(map_msg, saveParameters));

  saveParameters.free_thresh = 0.7;
  saveParameters.occupied_thresh = 0.2;
  ASSERT_FALSE(saveMapToFile(map_msg, saveParameters));

  saveParameters.free_thresh = 0.0;
  saveParameters.occupied_thresh = 0.0;
  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));

  saveParameters.map_file_name = path("/invalid_path") / path(g_valid_map_name);
  ASSERT_FALSE(saveMapToFile(map_msg, saveParameters));
}

// Load valid YAML file and check for consistency
TEST_F(MapIOTester, loadValidYAML)
{
  LoadParameters loadParameters;
  ASSERT_NO_THROW(loadParameters = loadMapYaml(path(TEST_DIR) / path(g_valid_yaml_file)));

  LoadParameters refLoadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_png_file), refLoadParameters);
  ASSERT_EQ(loadParameters.image_file_name, refLoadParameters.image_file_name);
  ASSERT_FLOAT_EQ(loadParameters.resolution, refLoadParameters.resolution);
  ASSERT_EQ(loadParameters.origin, refLoadParameters.origin);
  ASSERT_FLOAT_EQ(loadParameters.free_thresh, refLoadParameters.free_thresh);
  ASSERT_FLOAT_EQ(loadParameters.occupied_thresh, refLoadParameters.occupied_thresh);
  ASSERT_EQ(loadParameters.mode, refLoadParameters.mode);
  ASSERT_EQ(loadParameters.negate, refLoadParameters.negate);
}

// Try to load invalid YAML file
TEST_F(MapIOTester, loadInvalidYAML)
{
  LoadParameters loadParameters;
  ASSERT_ANY_THROW(loadParameters = loadMapYaml(path(TEST_DIR) / path("invalid_file.yaml")));
}

TEST(HomeUserExpanderTestSuite, homeUserExpanderShouldNotChangeInputStringWhenShorterThanTwo)
{
  const std::string emptyFileName{};
  ASSERT_EQ(emptyFileName, expand_user_home_dir_if_needed(emptyFileName, "/home/user"));
}

TEST(
  HomeUserExpanderTestSuite,
  homeUserExpanderShouldNotChangeInputStringWhenInputStringDoesNotStartWithHomeSequence)
{
  const std::string fileName{"valid_file.yaml"};
  ASSERT_EQ(fileName, expand_user_home_dir_if_needed(fileName, "/home/user"));
}

TEST(HomeUserExpanderTestSuite, homeUserExpanderShouldNotChangeInputStringWhenHomeVariableNotFound)
{
  const std::string fileName{"~/valid_file.yaml"};
  ASSERT_EQ(fileName, expand_user_home_dir_if_needed(fileName, ""));
}

TEST(HomeUserExpanderTestSuite, homeUserExpanderShouldExpandHomeSequenceWhenHomeVariableSet)
{
  const std::string fileName{"~/valid_file.yaml"};
  const std::string expectedOutputFileName{"/home/user/valid_file.yaml"};
  ASSERT_EQ(expectedOutputFileName, expand_user_home_dir_if_needed(fileName, "/home/user"));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
