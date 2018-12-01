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

#include "nav2_map_server/occ_grid_loader.hpp"
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

class TestMapLoader : public nav2_map_server::OccGridLoader
{
public:
  explicit TestMapLoader(rclcpp::Node * node, YAML::Node & doc)
  : OccGridLoader(node, doc)
  {
  }

  nav_msgs::msg::OccupancyGrid getOccupancyGrid()
  {
    return msg_;
  }
};

class MapLoaderTest : public ::testing::Test
{
public:
  MapLoaderTest()
  {
    // Set up a fake YAML document with the required fields
    doc_["resolution"] = "0.1";
    doc_["origin"][0] = "2.0";
    doc_["origin"][1] = "3.0";
    doc_["origin"][2] = "1.0";
    doc_["negate"] = "0";
    doc_["occupied_thresh"] = "0.65";
    doc_["free_thresh"] = "0.196";

    node_ = std::make_shared<rclcpp::Node>("map_server");
    map_loader_ = new TestMapLoader(node_.get(), doc_);
  }

protected:
  rclcpp::Node::SharedPtr node_;
  TestMapLoader * map_loader_;
  YAML::Node doc_;
};

/* Try to load a valid PNG file.  Succeeds if no exception is thrown, and if
 * the loaded image matches the known dimensions and content of the file.
 *
 * This test can fail on OS X, due to an apparent limitation of the
 * underlying SDL_Image library. */

TEST_F(MapLoaderTest, loadValidPNG)
{
  auto test_png = path(TEST_DIR) / path(g_valid_png_file);

  ASSERT_NO_THROW(map_loader_->loadMapFromFile(test_png.string()));
  nav_msgs::msg::OccupancyGrid map_msg = map_loader_->getOccupancyGrid();

  EXPECT_FLOAT_EQ(map_msg.info.resolution, g_valid_image_res);
  EXPECT_EQ(map_msg.info.width, g_valid_image_width);
  EXPECT_EQ(map_msg.info.height, g_valid_image_height);
  for (unsigned int i = 0; i < map_msg.info.width * map_msg.info.height; i++) {
    EXPECT_EQ(g_valid_image_content[i], map_msg.data[i]);
  }
}

/* Try to load a valid BMP file.  Succeeds if no exception is thrown, and if
 * the loaded image matches the known dimensions and content of the file. */

TEST_F(MapLoaderTest, loadValidBMP)
{
  auto test_bmp = path(TEST_DIR) / path(g_valid_bmp_file);


  ASSERT_NO_THROW(map_loader_->loadMapFromFile(test_bmp.string()));
  nav_msgs::msg::OccupancyGrid map_msg = map_loader_->getOccupancyGrid();

  EXPECT_FLOAT_EQ(map_msg.info.resolution, g_valid_image_res);
  EXPECT_EQ(map_msg.info.width, g_valid_image_width);
  EXPECT_EQ(map_msg.info.height, g_valid_image_height);
  for (unsigned int i = 0; i < map_msg.info.width * map_msg.info.height; i++) {
    EXPECT_EQ(g_valid_image_content[i], map_msg.data[i]);
  }
}

/* Try to load an invalid file.  Succeeds if a std::runtime exception is thrown */

TEST_F(MapLoaderTest, loadInvalidFile)
{
  auto test_invalid = path(TEST_DIR) / path("foo");
  ASSERT_THROW(map_loader_->loadMapFromFile(test_invalid.string()), std::runtime_error);
}
