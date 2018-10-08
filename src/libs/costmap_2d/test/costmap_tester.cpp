/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <tf2_ros/transform_listener.h>

namespace costmap_2d {

class CostmapTester : public testing::Test {
  public:
    CostmapTester(tf2_ros::Buffer& tf);
    void checkConsistentCosts();
    void compareCellToNeighbors(costmap_2d::Costmap2D& costmap, unsigned int x, unsigned int y);
    void compareCells(costmap_2d::Costmap2D& costmap, 
        unsigned int x, unsigned int y, unsigned int nx, unsigned int ny);
    virtual void TestBody(){}

  private:
    costmap_2d::Costmap2DROS costmap_ros_;
};

CostmapTester::CostmapTester(tf2_ros::Buffer& tf): costmap_ros_("test_costmap", tf){}

void CostmapTester::checkConsistentCosts(){
  costmap_2d::Costmap2D* costmap = costmap_ros_.getCostmap();

  //get a copy of the costmap contained by our ros wrapper
  costmap->saveMap("costmap_test.pgm");

  //loop through the costmap and check for any unexpected drop-offs in costs
  for(unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i){
    for(unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j){
      compareCellToNeighbors(*costmap, i, j);
    }
  }
}

void CostmapTester::compareCellToNeighbors(costmap_2d::Costmap2D& costmap, unsigned int x, unsigned int y){
  //we'll compare the cost of this cell with that of its eight neighbors to see if they're reasonable
  for(int offset_x = -1; offset_x <= 1; ++offset_x){
    for(int offset_y = -1; offset_y <= 1; ++offset_y){
      int nx = x + offset_x;
      int ny = y + offset_y;

      //check to make sure that the neighbor cell is a legal one
      if(nx >= 0 && nx < (int)costmap.getSizeInCellsX() && ny >=0 && ny < (int)costmap.getSizeInCellsY()){
        compareCells(costmap, x, y, nx, ny);
      }
    }
  }
}

//for all lethal and inscribed costs, we'll make sure that their neighbors have the cost values we'd expect
void CostmapTester::compareCells(costmap_2d::Costmap2D& costmap, unsigned int x, unsigned int y, unsigned int nx, unsigned int ny){
  double cell_distance = hypot(static_cast<int>(x-nx), static_cast<int>(y-ny));

  unsigned char cell_cost = costmap.getCost(x, y);
  unsigned char neighbor_cost = costmap.getCost(nx, ny);

  if(cell_cost == costmap_2d::LETHAL_OBSTACLE){
    //if the cell is a lethal obstacle, then we know that all its neighbors should have equal or slighlty less cost
    unsigned char expected_lowest_cost = 0; // ################costmap.computeCost(cell_distance);
    EXPECT_TRUE(neighbor_cost >= expected_lowest_cost || (cell_distance > 0 /*costmap.cell_inflation_radius_*/ && neighbor_cost == costmap_2d::FREE_SPACE));
  }
  else if(cell_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
    //the furthest valid distance from an obstacle is the inscribed radius plus the cell distance away
    double furthest_valid_distance = 0; // ################costmap.cell_inscribed_radius_ + cell_distance + 1;
    unsigned char expected_lowest_cost = 0; // ################costmap.computeCost(furthest_valid_distance);
    if(neighbor_cost < expected_lowest_cost){
      ROS_ERROR("Cell cost (%d, %d): %d, neighbor cost (%d, %d): %d, expected lowest cost: %d, cell distance: %.2f, furthest valid distance: %.2f",
          x, y, cell_cost, nx, ny, neighbor_cost, expected_lowest_cost, cell_distance, furthest_valid_distance);
      ROS_ERROR("Cell: (%d, %d), Neighbor: (%d, %d)", x, y, nx, ny);
      costmap.saveMap("failing_costmap.pgm");
    }
    EXPECT_TRUE(neighbor_cost >= expected_lowest_cost || (furthest_valid_distance > 0/* costmap.cell_inflation_radius_ */&& neighbor_cost == costmap_2d::FREE_SPACE));
  }
}
};

costmap_2d::CostmapTester* map_tester = NULL;

TEST(CostmapTester, checkConsistentCosts){
  map_tester->checkConsistentCosts();
}

void testCallback(const ros::TimerEvent& e){
  int test_result = RUN_ALL_TESTS();
  ROS_INFO("gtest return value: %d", test_result);
  ros::shutdown();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "costmap_tester_node");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  tf2_ros::Buffer tf(ros::Duration(10));
  tf2_ros::TransformListener tfl(tf);
  map_tester = new costmap_2d::CostmapTester(tf);

  double wait_time;
  private_nh.param("wait_time", wait_time, 30.0);
  ros::Timer timer = n.createTimer(ros::Duration(wait_time), testCallback);

  ros::spin();

  return(0);
}
