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
#include <experimental/filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

#include "yaml-cpp/yaml.h"
#include "nav2_map_server/map_io.hpp"
#include "nav2_map_server/map_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "test_constants/test_constants.h"

#define TEST_DIR TEST_DIRECTORY

using namespace std; // NOLINT
using std::experimental::filesystem::path;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

class MapIOTester : public ::testing::Test
{};

// Try to load a valid PNG file.  Succeeds if no exception is thrown, and if
// the loaded image matches the known dimensions and content of the file.

TEST_F(MapIOTester, loadValidPNG)
{
  auto test_png = path(TEST_DIR) / path(g_valid_png_file);

  nav2_map_server::LoadParameters loadParameters;
  loadParameters.image_file_name = test_png;
  loadParameters.resolution = g_valid_image_res;
  loadParameters.origin[0] = 0;
  loadParameters.origin[1] = 0;
  loadParameters.origin[2] = 0;
  loadParameters.free_thresh = 0.196;
  loadParameters.occupied_thresh = 0.65;
  loadParameters.mode = nav2_map_server::MapMode::Trinary;
  loadParameters.negate = 0;

  // In order to loadMapFromFile without going through the Configure and Activate states,
  // the msg_ member must be initialized
  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  EXPECT_FLOAT_EQ(map_msg.info.resolution, g_valid_image_res);
  EXPECT_EQ(map_msg.info.width, g_valid_image_width);
  EXPECT_EQ(map_msg.info.height, g_valid_image_height);
  for (unsigned int i = 0; i < map_msg.info.width * map_msg.info.height; i++) {
    EXPECT_EQ(g_valid_image_content[i], map_msg.data[i]);
  }
}

// Try to load a valid BMP file.  Succeeds if no exception is thrown, and if
// the loaded image matches the known dimensions and content of the file.

TEST_F(MapIOTester, loadValidBMP)
{
  auto test_bmp = path(TEST_DIR) / path(g_valid_bmp_file);

  nav2_map_server::LoadParameters loadParameters;
  loadParameters.image_file_name = test_bmp;
  loadParameters.resolution = g_valid_image_res;
  loadParameters.origin[0] = 0;
  loadParameters.origin[1] = 0;
  loadParameters.origin[2] = 0;
  loadParameters.free_thresh = 0.196;
  loadParameters.occupied_thresh = 0.65;
  loadParameters.mode = nav2_map_server::MapMode::Trinary;
  loadParameters.negate = 0;

  // In order to loadMapFromFile without going through the Configure and Activate states,
  // the msg_ member must be initialized
  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  EXPECT_FLOAT_EQ(map_msg.info.resolution, g_valid_image_res);
  EXPECT_EQ(map_msg.info.width, g_valid_image_width);
  EXPECT_EQ(map_msg.info.height, g_valid_image_height);
  for (unsigned int i = 0; i < map_msg.info.width * map_msg.info.height; i++) {
    EXPECT_EQ(g_valid_image_content[i], map_msg.data[i]);
  }
}

// Try to load an invalid file.  Succeeds if a std::runtime exception is thrown

TEST_F(MapIOTester, loadInvalidFile)
{
  auto test_invalid = path(TEST_DIR) / path("foo");

  nav2_map_server::LoadParameters loadParameters;
  loadParameters.image_file_name = test_invalid;
  loadParameters.resolution = g_valid_image_res;
  loadParameters.origin[0] = 0;
  loadParameters.origin[1] = 0;
  loadParameters.origin[2] = 0;
  loadParameters.free_thresh = 0.196;
  loadParameters.occupied_thresh = 0.65;
  loadParameters.mode = nav2_map_server::MapMode::Trinary;
  loadParameters.negate = 0;

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_ANY_THROW(loadMapFromFile(loadParameters, map_msg));
}
