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

#include <stdexcept> // for std::runtime_error
#include <gtest/gtest.h>
#include "test_constants.h"
#include "map_server/map_reps/occupancy_grid.h"



class OccGridTest : public OccGridLoader
{

public:

  OccGridTest(){}
  ~OccGridTest(){}
 
  nav_msgs::srv::GetMap::Response getMap(){return occ_resp_;}

};




class MapServerTest : public ::testing::Test
{

protected:
  OccGridTest * OccTest;

  MapServerTest()
  {
    OccTest = new OccGridTest;
  }

};


/* Try to load a valid PNG file.  Succeeds if no exception is thrown, and if
 * the loaded image matches the known dimensions and content of the file.
 *
 * This test can fail on OS X, due to an apparent limitation of the
 * underlying SDL_Image library. */

TEST_F(MapServerTest, loadValidPNG)
{
  try
  {
    nav_msgs::srv::GetMap::Response map_resp;
    std::string path = "/home/brian/ros2_overlay_ws/src/navigation2/src/map_server/test/testmap.png";
    OccTest->loadMapInfoFromFile("/home/brian/ros2_overlay_ws/src/navigation2/src/map_server/test/testmap.yaml");
    OccTest->loadMapFromFile(path);
    OccTest->setMap();
    map_resp = OccTest->getMap();

    EXPECT_FLOAT_EQ(map_resp.map.info.resolution, g_valid_image_res);
    EXPECT_EQ(map_resp.map.info.width, g_valid_image_width);
    EXPECT_EQ(map_resp.map.info.height, g_valid_image_height);
    for(unsigned int i=0; i < map_resp.map.info.width * map_resp.map.info.height; i++)
      EXPECT_EQ(g_valid_image_content[i], map_resp.map.data[i]);
  }
  catch(...)
  {
    ADD_FAILURE() << "Uncaught exception : " << "This is OK on OS X";
  }

}

/* Try to load a valid BMP file.  Succeeds if no exception is thrown, and if
 * the loaded image matches the known dimensions and content of the file. */

TEST_F(MapServerTest, loadValidBMP)
{
  try
  {
    nav_msgs::srv::GetMap::Response map_resp;
    std::string path = "/home/brian/ros2_overlay_ws/src/navigation2/src/map_server/test/testmap.bmp";
    OccTest->loadMapInfoFromFile("/home/brian/ros2_overlay_ws/src/navigation2/src/map_server/test/testmap.yaml");
    OccTest->loadMapFromFile(path);
    OccTest->setMap();
    map_resp = OccTest->getMap();

    EXPECT_FLOAT_EQ(map_resp.map.info.resolution, g_valid_image_res);
    EXPECT_EQ(map_resp.map.info.width, g_valid_image_width);
    EXPECT_EQ(map_resp.map.info.height, g_valid_image_height);
    for(unsigned int i=0; i < map_resp.map.info.width * map_resp.map.info.height; i++)
      EXPECT_EQ(g_valid_image_content[i], map_resp.map.data[i]);
  }
  catch(...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }

}


TEST_F(MapServerTest, loadInvalidFile)
{
  try
  {
    std::string path = "/home/brian/ros2_overlay_ws/src/navigation2/src/map_server/test/foo.png";
    OccTest->loadMapInfoFromFile("/home/brian/ros2_overlay_ws/src/navigation2/src/map_server/test/testmap.yaml");
    OccTest->loadMapFromFile(path);
    //OccTest->setMap();
    //map_resp = OccTest->getMap();

  }
  catch(std::runtime_error &e)
  {
    SUCCEED();
    return;
  }
  catch(...)
  {
    FAIL() << "Uncaught exception";
  }
  ADD_FAILURE() << "Didn't throw exception as expected";

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
