/*
 * footprint_helper_test.cpp
 *
 *  Created on: May 2, 2012
 *      Author: tkruse
 */

#include <gtest/gtest.h>

#include <vector>

#include <base_local_planner/footprint_helper.h>

#include <base_local_planner/map_grid.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>

#include "wavefront_map_accessor.h"

namespace base_local_planner {


class FootprintHelperTest : public testing::Test {
public:
  FootprintHelper fh;

  FootprintHelperTest() {
  }

  virtual void TestBody(){}

  void correctLineCells() {
    std::vector<base_local_planner::Position2DInt> footprint;
    fh.getLineCells(0, 10, 0, 10, footprint);
    EXPECT_EQ(11, footprint.size());
    EXPECT_EQ(footprint[0].x, 0);
    EXPECT_EQ(footprint[0].y, 0);
    EXPECT_EQ(footprint[5].x, 5);
    EXPECT_EQ(footprint[5].y, 5);
    EXPECT_EQ(footprint[10].x, 10);
    EXPECT_EQ(footprint[10].y, 10);
  }

  void correctFootprint(){
    MapGrid* mg = new MapGrid (10, 10);
    WavefrontMapAccessor* wa = new WavefrontMapAccessor(mg, .25);
    const costmap_2d::Costmap2D& map = *wa;

    std::vector<geometry_msgs::Point> footprint_spec;
    geometry_msgs::Point pt;
    //create a square footprint
    pt.x = 2;
    pt.y = 2;
    footprint_spec.push_back(pt);
    pt.x = 2;
    pt.y = -2;
    footprint_spec.push_back(pt);
    pt.x = -2;
    pt.y = -2;
    footprint_spec.push_back(pt);
    pt.x = -2;
    pt.y = 2;
    footprint_spec.push_back(pt);

    Eigen::Vector3f pos(4.5, 4.5, 0);
    //just create a basic footprint
    std::vector<base_local_planner::Position2DInt> footprint = fh.getFootprintCells(pos, footprint_spec, map, false);

    EXPECT_EQ(20, footprint.size());
    //we expect the front line to be first
    EXPECT_EQ(footprint[0].x, 6); EXPECT_EQ(footprint[0].y, 6);
    EXPECT_EQ(footprint[1].x, 6); EXPECT_EQ(footprint[1].y, 5);
    EXPECT_EQ(footprint[2].x, 6); EXPECT_EQ(footprint[2].y, 4);
    EXPECT_EQ(footprint[3].x, 6); EXPECT_EQ(footprint[3].y, 3);
    EXPECT_EQ(footprint[4].x, 6); EXPECT_EQ(footprint[4].y, 2);

    //next the right line
    EXPECT_EQ(footprint[5].x, 6); EXPECT_EQ(footprint[5].y, 2);
    EXPECT_EQ(footprint[6].x, 5); EXPECT_EQ(footprint[6].y, 2);
    EXPECT_EQ(footprint[7].x, 4); EXPECT_EQ(footprint[7].y, 2);
    EXPECT_EQ(footprint[8].x, 3); EXPECT_EQ(footprint[8].y, 2);
    EXPECT_EQ(footprint[9].x, 2); EXPECT_EQ(footprint[9].y, 2);

    //next the back line
    EXPECT_EQ(footprint[10].x, 2); EXPECT_EQ(footprint[10].y, 2);
    EXPECT_EQ(footprint[11].x, 2); EXPECT_EQ(footprint[11].y, 3);
    EXPECT_EQ(footprint[12].x, 2); EXPECT_EQ(footprint[12].y, 4);
    EXPECT_EQ(footprint[13].x, 2); EXPECT_EQ(footprint[13].y, 5);
    EXPECT_EQ(footprint[14].x, 2); EXPECT_EQ(footprint[14].y, 6);

    //finally the left line
    EXPECT_EQ(footprint[15].x, 2); EXPECT_EQ(footprint[15].y, 6);
    EXPECT_EQ(footprint[16].x, 3); EXPECT_EQ(footprint[16].y, 6);
    EXPECT_EQ(footprint[17].x, 4); EXPECT_EQ(footprint[17].y, 6);
    EXPECT_EQ(footprint[18].x, 5); EXPECT_EQ(footprint[18].y, 6);
    EXPECT_EQ(footprint[19].x, 6); EXPECT_EQ(footprint[19].y, 6);


    pos = Eigen::Vector3f(4.5, 4.5, M_PI_2);
    //check that rotation of the footprint works
    footprint = fh.getFootprintCells(pos, footprint_spec, map, false);

    //first the left line
    EXPECT_EQ(footprint[0].x, 2); EXPECT_EQ(footprint[0].y, 6);
    EXPECT_EQ(footprint[1].x, 3); EXPECT_EQ(footprint[1].y, 6);
    EXPECT_EQ(footprint[2].x, 4); EXPECT_EQ(footprint[2].y, 6);
    EXPECT_EQ(footprint[3].x, 5); EXPECT_EQ(footprint[3].y, 6);
    EXPECT_EQ(footprint[4].x, 6); EXPECT_EQ(footprint[4].y, 6);

    //next the front line
    EXPECT_EQ(footprint[5].x, 6); EXPECT_EQ(footprint[5].y, 6);
    EXPECT_EQ(footprint[6].x, 6); EXPECT_EQ(footprint[6].y, 5);
    EXPECT_EQ(footprint[7].x, 6); EXPECT_EQ(footprint[7].y, 4);
    EXPECT_EQ(footprint[8].x, 6); EXPECT_EQ(footprint[8].y, 3);
    EXPECT_EQ(footprint[9].x, 6); EXPECT_EQ(footprint[9].y, 2);

    //next the right line
    EXPECT_EQ(footprint[10].x, 6); EXPECT_EQ(footprint[10].y, 2);
    EXPECT_EQ(footprint[11].x, 5); EXPECT_EQ(footprint[11].y, 2);
    EXPECT_EQ(footprint[12].x, 4); EXPECT_EQ(footprint[12].y, 2);
    EXPECT_EQ(footprint[13].x, 3); EXPECT_EQ(footprint[13].y, 2);
    EXPECT_EQ(footprint[14].x, 2); EXPECT_EQ(footprint[14].y, 2);

    //next the back line
    EXPECT_EQ(footprint[15].x, 2); EXPECT_EQ(footprint[15].y, 2);
    EXPECT_EQ(footprint[16].x, 2); EXPECT_EQ(footprint[16].y, 3);
    EXPECT_EQ(footprint[17].x, 2); EXPECT_EQ(footprint[17].y, 4);
    EXPECT_EQ(footprint[18].x, 2); EXPECT_EQ(footprint[18].y, 5);
    EXPECT_EQ(footprint[19].x, 2); EXPECT_EQ(footprint[19].y, 6);
  }

};


TEST(FootprintHelperTest, correctFootprint){
  FootprintHelperTest tct;
  tct.correctFootprint();
}

TEST(FootprintHelperTest, correctLineCells){
  FootprintHelperTest tct;
  tct.correctLineCells();
}

}
