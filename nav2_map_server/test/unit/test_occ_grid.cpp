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

#include "nav2_map_server/map_representations/occ_grid_server.hpp"
#include "test_constants/test_constants.h"

#define TEST_DIR TEST_DIRECTORY

using namespace std; // NOLINT
using std::experimental::filesystem::path;

class OccGridTest : public nav2_map_server::OccGridServer
{
public:
  OccGridTest() {}
  ~OccGridTest() {}

  nav_msgs::srv::GetMap::Response GetMap() {return occ_resp_;}
};

class MapServerTest : public ::testing::Test
{
public:
  MapServerTest()
  {
    OccTest = new OccGridTest;
    auto test_yaml = path(TEST_DIR) / path(g_valid_yaml_file);
    OccTest->LoadMapInfoFromFile(test_yaml.string());
  }

protected:
  OccGridTest * OccTest;
  nav_msgs::srv::GetMap::Response map_resp;
};

/* Try to load a valid PNG file.  Succeeds if no exception is thrown, and if
 * the loaded image matches the known dimensions and content of the file.
 *
 * This test can fail on OS X, due to an apparent limitation of the
 * underlying SDL_Image library. */

TEST_F(MapServerTest, loadValidPNG)
{
  auto test_png = path(TEST_DIR) / path(g_valid_png_file);
  ASSERT_NO_THROW(OccTest->LoadMapFromFile(test_png.string()));
  OccTest->SetMap();
  map_resp = OccTest->GetMap();

  EXPECT_FLOAT_EQ(map_resp.map.info.resolution, g_valid_image_res);
  EXPECT_EQ(map_resp.map.info.width, g_valid_image_width);
  EXPECT_EQ(map_resp.map.info.height, g_valid_image_height);
  for (unsigned int i = 0; i < map_resp.map.info.width * map_resp.map.info.height; i++) {
    EXPECT_EQ(g_valid_image_content[i], map_resp.map.data[i]);
  }
}

/* Try to load a valid BMP file.  Succeeds if no exception is thrown, and if
 * the loaded image matches the known dimensions and content of the file. */

TEST_F(MapServerTest, loadValidBMP)
{
  auto test_bmp = path(TEST_DIR) / path(g_valid_bmp_file);
  ASSERT_NO_THROW(OccTest->LoadMapFromFile(test_bmp.string()));
  OccTest->SetMap();
  map_resp = OccTest->GetMap();

  EXPECT_FLOAT_EQ(map_resp.map.info.resolution, g_valid_image_res);
  EXPECT_EQ(map_resp.map.info.width, g_valid_image_width);
  EXPECT_EQ(map_resp.map.info.height, g_valid_image_height);
  for (unsigned int i = 0; i < map_resp.map.info.width * map_resp.map.info.height; i++) {
    EXPECT_EQ(g_valid_image_content[i], map_resp.map.data[i]);
  }
}

/* Try to load an invalid file.  Succeeds if a std::runtime exception is thrown */

TEST_F(MapServerTest, loadInvalidFile)
{
  auto test_invalid = path(TEST_DIR) / path("foo");
  ASSERT_THROW(OccTest->LoadMapFromFile(test_invalid.string()), std::runtime_error);
}
