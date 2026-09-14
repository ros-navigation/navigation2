// Copyright (c) 2026 Dexory

#include <limits>

#include "gtest/gtest.h"
#include "nav2_collision_monitor/triggering_cloud.hpp"

using nav2_collision_monitor::TriggeringCloud;
using nav2_collision_monitor::STOP;
using nav2_collision_monitor::DO_NOTHING;

TEST(TriggeringCloud, EmptyAndTaggedPoints)
{
  TriggeringCloud builder;
  builder.configure({"front", "rear"}, {"stop", "slow"});
  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  header.stamp.sec = 42;
  auto cloud = TriggeringCloud::create(header);
  EXPECT_EQ(cloud.header, header);
  EXPECT_EQ(cloud.width, 0u);
  EXPECT_EQ(cloud.height, 1u);
  EXPECT_EQ(cloud.point_step, 24u);
  const auto empty_cloud = cloud;
  builder.append(cloud, {}, "slow", STOP);
  EXPECT_EQ(cloud, empty_cloud);
  builder.append(cloud, {{1, 2, 3, "rear"}, {4, 5, 6, "front"}}, "slow", STOP);
  builder.append(cloud, {{1, 2, 3, "rear"}}, "stop", DO_NOTHING);
  const auto populated_cloud = cloud;
  builder.append(cloud, {}, "stop", DO_NOTHING);
  EXPECT_EQ(cloud, populated_cloud);
  ASSERT_EQ(cloud.width, 3u);
  EXPECT_EQ(cloud.row_step, 72u);
  sensor_msgs::PointCloud2ConstIterator<uint32_t> source(cloud, "source_id");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> polygon(cloud, "polygon_id");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> action(cloud, "action_type");
  sensor_msgs::PointCloud2ConstIterator<float> height(cloud, "z");
  EXPECT_EQ(source[0], 1u);
  EXPECT_EQ(*polygon, 1u);
  EXPECT_EQ(*action, static_cast<uint32_t>(STOP));
  EXPECT_FLOAT_EQ(*height, 3);
  ++source;
  ++polygon;
  ++action;
  EXPECT_EQ(*source, 0u);
  ++source;
  ++polygon;
  ++action;
  EXPECT_EQ(*source, 1u);
  EXPECT_EQ(*polygon, 0u);
  EXPECT_EQ(*action, static_cast<uint32_t>(DO_NOTHING));
}

TEST(TriggeringCloud, DropsNonfiniteAndResetsMapping)
{
  TriggeringCloud builder;
  builder.configure({"old"}, {"old"});
  builder.configure({"new"}, {"new"});
  auto cloud = TriggeringCloud::create(std_msgs::msg::Header{});
  builder.append(cloud, {{0, 0, std::numeric_limits<double>::infinity(), "new"},
      {0, 0, 1, "old"}}, "old", STOP);
  ASSERT_EQ(cloud.width, 1u);
  EXPECT_TRUE(cloud.is_dense);
  sensor_msgs::PointCloud2ConstIterator<uint32_t> source(cloud, "source_id");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> polygon(cloud, "polygon_id");
  EXPECT_EQ(*source, std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(*polygon, std::numeric_limits<uint32_t>::max());
}
