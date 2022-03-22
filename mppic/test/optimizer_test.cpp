#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/goal_checker.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "mppic/optimizer.hpp"
#include "mppic/motion_models.hpp"

#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

TEST(MPPIOptimizer, OptimizerTestDiffFootprint)
{
  bool consider_footprint = true;
  std::string motion_model = "DiffDrive";

  // Settings
  TestCostmapSettings cost_map_settings{};
  TestOptimizerSettings optimizer_settings{12, 80, 5.0, motion_model, consider_footprint};

  const double path_step = cost_map_settings.resolution;
  TestPose start_pose = cost_map_settings.getCenterPose();
  TestPathSettings path_settings{start_pose, 8, path_step, path_step};

  print_info(optimizer_settings, path_settings);

  auto costmap_ros = getDummyCostmapRos(cost_map_settings);
  auto node = getDummyNode(optimizer_settings);
  auto optimizer = getDummyOptimizer(node, costmap_ros);

  // setup costmap
  auto costmap = costmap_ros->getCostmap();
  {
    costmap_ros->setRobotFootprint(getDummySquareFootprint(0.01));
    const unsigned int obst_center_x = cost_map_settings.cells_x / 2;
    const unsigned int obst_center_y = cost_map_settings.cells_y / 2 + 1;
    TestObstaclesSettings obs_settings_1{obst_center_x - 4, obst_center_y, 4, 255};
    TestObstaclesSettings obs_settings_2{obst_center_x + 4, obst_center_y, 4, 255};
    TestObstaclesSettings obs_settings_3{obst_center_x + 12, obst_center_y, 4, 255};
    addObstacle(costmap, obs_settings_1);
    addObstacle(costmap, obs_settings_2);
    addObstacle(costmap, obs_settings_3);
  }

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);

  nav2_core::GoalChecker * dummy_goal_checker{nullptr};
  EXPECT_NO_THROW(optimizer.evalControl(pose, velocity, path, dummy_goal_checker));
  auto trajectory = optimizer.evalTrajectoryFromControlSequence(pose, velocity);
  auto goal_point = path.poses.back();
#ifdef TEST_DEBUG_INFO
  printMapWithTrajectoryAndGoal(*costmap, trajectory, goal_point);
#endif
  EXPECT_TRUE(!inCollision(trajectory, *costmap));
  EXPECT_TRUE(isGoalReached(trajectory, *costmap, goal_point));
}

TEST(MPPIOptimizer, OptimizerTestOmniCircle)
{
  bool consider_footprint = false;
  std::string motion_model = "Omni";

  // Settings
  TestCostmapSettings cost_map_settings{};
  TestOptimizerSettings optimizer_settings{12, 80, 5.0, motion_model, consider_footprint};

  const double path_step = cost_map_settings.resolution;
  TestPose start_pose = cost_map_settings.getCenterPose();
  TestPathSettings path_settings{start_pose, 8, path_step, path_step};

  print_info(optimizer_settings, path_settings);

  auto costmap_ros = getDummyCostmapRos(cost_map_settings);
  auto node = getDummyNode(optimizer_settings);
  auto optimizer = getDummyOptimizer(node, costmap_ros);

  // setup costmap
  auto costmap = costmap_ros->getCostmap();
  {
    costmap_ros->setRobotFootprint(getDummySquareFootprint(0.01));
    const unsigned int obst_center_x = cost_map_settings.cells_x / 2;
    const unsigned int obst_center_y = cost_map_settings.cells_y / 2 + 1;
    TestObstaclesSettings obs_settings_1{obst_center_x - 4, obst_center_y, 4, 255};
    TestObstaclesSettings obs_settings_2{obst_center_x + 4, obst_center_y, 4, 255};
    TestObstaclesSettings obs_settings_3{obst_center_x + 12, obst_center_y, 4, 255};
    addObstacle(costmap, obs_settings_1);
    addObstacle(costmap, obs_settings_2);
    addObstacle(costmap, obs_settings_3);
  }

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);

  nav2_core::GoalChecker * dummy_goal_checker{nullptr};
  EXPECT_NO_THROW(optimizer.evalControl(pose, velocity, path, dummy_goal_checker));
  auto trajectory = optimizer.evalTrajectoryFromControlSequence(pose, velocity);
  auto goal_point = path.poses.back();
#ifdef TEST_DEBUG_INFO
  printMapWithTrajectoryAndGoal(*costmap, trajectory, goal_point);
#endif
  EXPECT_TRUE(!inCollision(trajectory, *costmap));
  EXPECT_TRUE(isGoalReached(trajectory, *costmap, goal_point));
}
