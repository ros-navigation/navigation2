/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/
#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <utility>

#include <base_local_planner/map_cell.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/trajectory_planner.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>
#include <math.h>

#include <geometry_msgs/Point.h>
#include <base_local_planner/Position2DInt.h>

#include "wavefront_map_accessor.h"

using namespace std;

namespace base_local_planner {

class TrajectoryPlannerTest : public testing::Test {
  public:
    TrajectoryPlannerTest(MapGrid* g, WavefrontMapAccessor* wave, const costmap_2d::Costmap2D& map, std::vector<geometry_msgs::Point> footprint_spec);
    void correctFootprint();
    void footprintObstacles();
    void checkGoalDistance();
    void checkPathDistance();
    virtual void TestBody(){}

    MapGrid* map_;
    WavefrontMapAccessor* wa;
    CostmapModel cm;
    TrajectoryPlanner tc;
};

TrajectoryPlannerTest::TrajectoryPlannerTest(MapGrid* g, WavefrontMapAccessor* wave, const costmap_2d::Costmap2D& map, std::vector<geometry_msgs::Point> footprint_spec)
: map_(g), wa(wave), cm(map), tc(cm, map, footprint_spec, 0.0, 1.0, 1.0, 1.0, 1.0, 2.0)
{}



void TrajectoryPlannerTest::footprintObstacles(){
  //place an obstacle
  map_->operator ()(4, 6).target_dist = 1;
  wa->synchronize();
  EXPECT_EQ(wa->getCost(4,6), costmap_2d::LETHAL_OBSTACLE);
  Trajectory traj(0, 0, 0, 0.1, 30);
  //tc->generateTrajectory(4.5, 4.5, M_PI_2, 0, 0, 0, 4, 0, 0, 4, 0, 0, DBL_MAX, traj, 2, 30);
  tc.generateTrajectory(4.5, 4.5, M_PI_2, 0, 0, 0, 4, 0, 0, 4, 0, 0, DBL_MAX, traj);
  //we expect this path to hit the obstacle
  EXPECT_FLOAT_EQ(traj.cost_, -1.0);

  //place a wall next to the footprint of the robot
  tc.path_map_(7, 1).target_dist = 1;
  tc.path_map_(7, 3).target_dist = 1;
  tc.path_map_(7, 4).target_dist = 1;
  tc.path_map_(7, 5).target_dist = 1;
  tc.path_map_(7, 6).target_dist = 1;
  tc.path_map_(7, 7).target_dist = 1;
  wa->synchronize();

  //try to rotate into it
  //tc->generateTrajectory(4.5, 4.5, M_PI_2, 0, 0, 0, 0, 0, M_PI_2, 0, 0, M_PI_4, 100, traj, 2, 30);
  tc.generateTrajectory(4.5, 4.5, M_PI_2, 0, 0, 0, 0, 0, M_PI_2, 0, 0, M_PI_4, 100, traj);
  //we expect this path to hit the obstacle
  EXPECT_FLOAT_EQ(traj.cost_, -1.0);
}

void TrajectoryPlannerTest::checkGoalDistance(){
  //let's box a cell in and make sure that its distance gets set to max
  map_->operator ()(1, 2).target_dist = 1;
  map_->operator ()(1, 1).target_dist = 1;
  map_->operator ()(1, 0).target_dist = 1;
  map_->operator ()(2, 0).target_dist = 1;
  map_->operator ()(3, 0).target_dist = 1;
  map_->operator ()(3, 1).target_dist = 1;
  map_->operator ()(3, 2).target_dist = 1;
  map_->operator ()(2, 2).target_dist = 1;
  wa->synchronize();

  //set a goal
  tc.path_map_.resetPathDist();
  queue<MapCell*> target_dist_queue;
  MapCell& current = tc.path_map_(4, 9);
  current.target_dist = 0.0;
  current.target_mark = true;
  target_dist_queue.push(&current);
  tc.path_map_.computeTargetDistance(target_dist_queue, tc.costmap_);

  EXPECT_FLOAT_EQ(tc.path_map_(4, 8).target_dist, 1.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 7).target_dist, 2.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 6).target_dist, 100.0); //there's an obstacle here placed above
  EXPECT_FLOAT_EQ(tc.path_map_(4, 5).target_dist, 6.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 4).target_dist, 7.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 3).target_dist, 8.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 2).target_dist, 9.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 1).target_dist, 10.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 0).target_dist, 11.0);
  EXPECT_FLOAT_EQ(tc.path_map_(5, 8).target_dist, 2.0);
  EXPECT_FLOAT_EQ(tc.path_map_(9, 4).target_dist, 10.0);

  //check the boxed in cell
  EXPECT_FLOAT_EQ(100.0, tc.path_map_(2, 2).target_dist);

}

void TrajectoryPlannerTest::checkPathDistance(){
  tc.path_map_.resetPathDist();
  queue<MapCell*> target_dist_queue;
  MapCell& current = tc.path_map_(4, 9);
  current.target_dist = 0.0;
  current.target_mark = true;
  target_dist_queue.push(&current);
  tc.path_map_.computeTargetDistance(target_dist_queue, tc.costmap_);

  EXPECT_FLOAT_EQ(tc.path_map_(4, 8).target_dist, 1.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 7).target_dist, 2.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 6).target_dist, 100.0); //there's an obstacle here placed above
  EXPECT_FLOAT_EQ(tc.path_map_(4, 5).target_dist, 6.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 4).target_dist, 7.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 3).target_dist, 8.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 2).target_dist, 9.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 1).target_dist, 10.0);
  EXPECT_FLOAT_EQ(tc.path_map_(4, 0).target_dist, 11.0);
  EXPECT_FLOAT_EQ(tc.path_map_(5, 8).target_dist, 2.0);
  EXPECT_FLOAT_EQ(tc.path_map_(9, 4).target_dist, 10.0);

  //check the boxed in cell
  EXPECT_FLOAT_EQ(tc.path_map_(2, 2).target_dist, 100.0);

}


TrajectoryPlannerTest* tct = NULL;

TrajectoryPlannerTest* setup_testclass_singleton() {
  if (tct == NULL) {
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

    tct = new base_local_planner::TrajectoryPlannerTest(mg, wa, map, footprint_spec);
  }
  return tct;
}

//make sure that trajectories that intersect obstacles are invalidated
TEST(TrajectoryPlannerTest, footprintObstacles){
  TrajectoryPlannerTest* tct = setup_testclass_singleton();
  tct->footprintObstacles();
}

//make sure that goal distance is being computed as expected
TEST(TrajectoryPlannerTest, checkGoalDistance){
  TrajectoryPlannerTest* tct = setup_testclass_singleton();
  tct->checkGoalDistance();
}

//make sure that path distance is being computed as expected
TEST(TrajectoryPlannerTest, checkPathDistance){
  TrajectoryPlannerTest* tct = setup_testclass_singleton();
  tct->checkPathDistance();
}

}; //namespace
