/*
 * map_grid_test.cpp
 *
 *  Created on: May 2, 2012
 *      Author: tkruse
 */
#include <queue>

#include <gtest/gtest.h>

#include <base_local_planner/map_grid.h>
#include <base_local_planner/map_cell.h>

#include "wavefront_map_accessor.h"

namespace base_local_planner {

TEST(MapGridTest, initNull){
  MapGrid map_grid;
  EXPECT_EQ(0, map_grid.size_x_);
  EXPECT_EQ(0, map_grid.size_y_);
}

TEST(MapGridTest, operatorBrackets){
  MapGrid map_grid(10, 10);
  map_grid(3, 5).target_dist = 5;
  EXPECT_EQ(5, map_grid.getCell(3, 5).target_dist);
}

TEST(MapGridTest, copyConstructor){
  MapGrid map_grid(10, 10);
  map_grid(3, 5).target_dist = 5;
  MapGrid map_grid2;
  map_grid2 = map_grid;
  EXPECT_EQ(5, map_grid(3, 5).target_dist);
}

TEST(MapGridTest, getIndex){
  MapGrid map_grid(10, 10);
  EXPECT_EQ(53, map_grid.getIndex(3, 5));
}

TEST(MapGridTest, reset){
  MapGrid map_grid(10, 10);
  map_grid(0, 0).target_dist = 1;
  map_grid(0, 0).target_mark = true;
  map_grid(0, 0).within_robot = true;
  map_grid(3, 5).target_dist = 1;
  map_grid(3, 5).target_mark = true;
  map_grid(3, 5).within_robot = true;
  map_grid(9, 9).target_dist = 1;
  map_grid(9, 9).target_mark = true;
  map_grid(9, 9).within_robot = true;
  EXPECT_EQ(1, map_grid(0, 0).target_dist);
  EXPECT_EQ(true, map_grid(0, 0).target_mark);
  EXPECT_EQ(true, map_grid(0, 0).within_robot);
  EXPECT_EQ(1, map_grid(3, 5).target_dist);
  EXPECT_EQ(true, map_grid(3, 5).target_mark);
  EXPECT_EQ(true, map_grid(3, 5).within_robot);
  EXPECT_EQ(1, map_grid(9, 9).target_dist);
  EXPECT_EQ(true, map_grid(9, 9).target_mark);
  EXPECT_EQ(true, map_grid(9, 9).within_robot);

  map_grid.resetPathDist();

  EXPECT_EQ(map_grid.unreachableCellCosts(), map_grid(9, 9).target_dist);
  EXPECT_EQ(false, map_grid(9, 9).target_mark);
  EXPECT_EQ(false, map_grid(9, 9).within_robot);
  EXPECT_EQ(map_grid.unreachableCellCosts(), map_grid(3, 5).target_dist);
  EXPECT_EQ(false, map_grid(3, 5).target_mark);
  EXPECT_EQ(false, map_grid(3, 5).within_robot);
  EXPECT_EQ(map_grid.unreachableCellCosts(), map_grid(0, 0).target_dist);
  EXPECT_EQ(false, map_grid(0, 0).target_mark);
  EXPECT_EQ(false, map_grid(0, 0).within_robot);
}


TEST(MapGridTest, properGridConstruction){
  MapGrid mg(10, 10);
  MapCell mc;

  for(int i = 0; i < 10; ++i){
    for(int j = 0; j < 10; ++j){
      EXPECT_FLOAT_EQ(mg(i, j).cx, i);
      EXPECT_FLOAT_EQ(mg(i, j).cy, j);
    }
  }
}

TEST(MapGridTest, sizeCheck){
  MapGrid mg(10, 10);
  MapCell mc;

  mg.sizeCheck(20, 25);

  for(int i = 0; i < 20; ++i){
    for(int j = 0; j < 25; ++j){
      EXPECT_FLOAT_EQ(mg(i, j).cx, i);
      EXPECT_FLOAT_EQ(mg(i, j).cy, j);
    }
  }
}

TEST(MapGridTest, adjustPlanEmpty){
  MapGrid mg(10, 10);
  const std::vector<geometry_msgs::PoseStamped> global_plan_in;
  std::vector<geometry_msgs::PoseStamped> global_plan_out;
  double resolution = 0;
  mg.adjustPlanResolution(global_plan_in, global_plan_out, resolution);
  EXPECT_EQ(0, global_plan_out.size());
}

TEST(MapGridTest, adjustPlan){
  MapGrid mg(10, 10);
  std::vector<geometry_msgs::PoseStamped> global_plan_in;
  std::vector<geometry_msgs::PoseStamped> global_plan_out;
  double resolution = 1;
  geometry_msgs::PoseStamped start;
  start.pose.position.x = 1;
  start.pose.position.y = 1;
  geometry_msgs::PoseStamped end;
  end.pose.position.x = 5;
  end.pose.position.y = 5;
  global_plan_in.push_back(start);
  global_plan_in.push_back(end);
  mg.adjustPlanResolution(global_plan_in, global_plan_out, resolution);
  
  EXPECT_EQ(1, global_plan_out[0].pose.position.x);
  EXPECT_EQ(1, global_plan_out[0].pose.position.y);
  EXPECT_EQ(5, global_plan_out.back().pose.position.x);
  EXPECT_EQ(5, global_plan_out.back().pose.position.y);

  for (unsigned int i = 1; i < global_plan_out.size(); ++i)
  {
    geometry_msgs::Point& p0 = global_plan_out[i - 1].pose.position;
    geometry_msgs::Point& p1 = global_plan_out[i].pose.position;
    double d = hypot(p0.x - p1.x, p0.y - p1.y);
    EXPECT_LT(d, resolution);
  }
}

TEST(MapGridTest, adjustPlan2){
  std::vector<geometry_msgs::PoseStamped> base_plan, result;

  // Push two points, at (0,0) and (0,1). Gap is 1 meter
  base_plan.push_back(geometry_msgs::PoseStamped());
  base_plan.push_back(geometry_msgs::PoseStamped());
  base_plan.back().pose.position.y = 1.0;

  // resolution >= 1, path won't change
  MapGrid::adjustPlanResolution(base_plan, result, 2.0);
  EXPECT_EQ(2, result.size());
  result.clear();
  MapGrid::adjustPlanResolution(base_plan, result, 1.0);
  EXPECT_EQ(2, result.size());
  result.clear();

  // 0.5 <= resolution < 1.0, one point should be added in the middle
  MapGrid::adjustPlanResolution(base_plan, result, 0.8);
  EXPECT_EQ(3, result.size());
  result.clear();
  MapGrid::adjustPlanResolution(base_plan, result, 0.5);
  EXPECT_EQ(3, result.size());
  result.clear();

  // 0.333 <= resolution < 0.5, two points should be added in the middle
  MapGrid::adjustPlanResolution(base_plan, result, 0.34);
  EXPECT_EQ(4, result.size());
  result.clear();

  // 0.25 <= resolution < 0.333, three points should be added in the middle
  MapGrid::adjustPlanResolution(base_plan, result, 0.32);
  EXPECT_EQ(5, result.size());
  result.clear();

  MapGrid::adjustPlanResolution(base_plan, result, 0.1);
  EXPECT_EQ(11, result.size());
  result.clear();
}

TEST(MapGridTest, distancePropagation){
  MapGrid mg(10, 10);

  WavefrontMapAccessor* wa = new WavefrontMapAccessor(&mg, .25);
  std::queue<MapCell*> dist_queue;
  mg.computeTargetDistance(dist_queue, *wa);
  EXPECT_EQ(false, mg(0, 0).target_mark);

  MapCell& mc = mg.getCell(0, 0);
  mc.target_dist = 0.0;
  mc.target_mark = true;
  dist_queue.push(&mc);
  mg.computeTargetDistance(dist_queue, *wa);
  EXPECT_EQ(true, mg(0, 0).target_mark);
  EXPECT_EQ(0.0,  mg(0, 0).target_dist);
  EXPECT_EQ(true, mg(1, 1).target_mark);
  EXPECT_EQ(2.0,  mg(1, 1).target_dist);
  EXPECT_EQ(true, mg(0, 4).target_mark);
  EXPECT_EQ(4.0,  mg(0, 4).target_dist);
  EXPECT_EQ(true, mg(4, 0).target_mark);
  EXPECT_EQ(4.0,  mg(4, 0).target_dist);
  EXPECT_EQ(true, mg(9, 9).target_mark);
  EXPECT_EQ(18.0, mg(9, 9).target_dist);
}

}
