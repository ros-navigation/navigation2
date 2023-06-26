#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_smac_planner/utils.hpp"

using namespace nav2_smac_planner;

TEST(transform_footprint_to_edges, test_basic)
{
  geometry_msgs::msg::Point p1;
  p1.x = 1.0;
  p1.y = 1.0;

  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = -1.0;

  geometry_msgs::msg::Point p3;
  p3.x = -1.0;
  p3.y = -1.0;

  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = 1.0;

  std::vector<geometry_msgs::msg::Point> footprint{p1, p2, p3, p4};
  std::vector<geometry_msgs::msg::Point> footprint_edges{p1, p2, p2, p3, p3, p4, p4, p1};

  auto result = transformFootprintToEdges(0.0, 0.0, 0.0, footprint);
  EXPECT_EQ(result.size(), 8u);

  for (size_t i = 0; i < result.size(); i++) {
    auto & p = result[i];
    auto & q = footprint_edges[i];
    EXPECT_EQ(p.x, q.x);
    EXPECT_EQ(p.y, q.y);
  }
}

TEST(transform_footprint_to_edges, test_transition_rotation)
{
  geometry_msgs::msg::Point p1;
  p1.x = 1.0;
  p1.y = 1.0;

  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = -1.0;

  geometry_msgs::msg::Point p3;
  p3.x = -1.0;
  p3.y = -1.0;

  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = 1.0;

  const double x0 = 1.0;
  const double y0 = 1.0;
  const double yaw0 = M_PI / 4;

  std::vector<geometry_msgs::msg::Point> footprint{p1, p2, p3, p4};

  // q1
  geometry_msgs::msg::Point q1;
  q1.x = 0.0 + x0;
  q1.y = sqrt(2) + y0;

  // q2
  geometry_msgs::msg::Point q2;
  q2.x = sqrt(2.0) + x0;
  q2.y = 0.0 + y0;

  // q3
  geometry_msgs::msg::Point q3;
  q3.x = 0.0 + x0;
  q3.y = -sqrt(2) + y0;

  // q4
  geometry_msgs::msg::Point q4;
  q4.x = -sqrt(2.0) + x0;
  q4.y = 0.0 + y0;

  std::vector<geometry_msgs::msg::Point> footprint_edges{q1, q2, q2, q3, q3, q4, q4, q1};
  auto result = transformFootprintToEdges(x0, y0, yaw0, footprint);
  EXPECT_EQ(result.size(), 8u);

  for (size_t i = 0; i < result.size(); i++) {
    auto & p = result[i];
    auto & q = footprint_edges[i];
    EXPECT_EQ(p.x, q.x);
    EXPECT_EQ(p.y, q.y);
  }
}
