// Copyright (c) 2020 Samsung Research Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"

using namespace std::chrono_literals;

typedef std::recursive_mutex mutex_t;

static const std::string FILTER_NAME = "keepout_filter";
static const std::string INFO_TOPIC = "costmap_filter_info";
static const std::string MASK_TOPIC = "mask";

class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher()
  : Node("costmap_filter_info_pub"), is_published_(false)
  {
    access_ = new mutex_t();

    publisher_ = this->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      INFO_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<nav2_msgs::msg::CostmapFilterInfo> msg =
      std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
    msg->type = 0;
    msg->map_mask_topic = MASK_TOPIC;
    msg->base = 0.0;
    msg->multiplier = 1.0;

    publisher_->publish(std::move(msg));

    subscription_ = this->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
      INFO_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&InfoPublisher::infoCallback, this, std::placeholders::_1));
  }

  ~InfoPublisher()
  {
    subscription_.reset();
    publisher_.reset();
    delete access_;
  }

  bool isPublished()
  {
    std::lock_guard<mutex_t> guard(*getMutex());
    return is_published_;
  }

  mutex_t * getMutex()
  {
    return access_;
  }

private:
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;
  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr subscription_;

  void infoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr)
  {
    std::lock_guard<mutex_t> guard(*getMutex());
    is_published_ = true;
  }

  bool is_published_;

  mutex_t * access_;
};  // InfoPublisher

class MaskPublisher : public rclcpp::Node
{
public:
  MaskPublisher(const nav_msgs::msg::OccupancyGrid & mask)
  : Node("mask_pub"), is_published_(false)
  {
    access_ = new mutex_t();

    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      MASK_TOPIC,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    publisher_->publish(mask);

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      MASK_TOPIC,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&MaskPublisher::mapCallback, this, std::placeholders::_1));
  }

  ~MaskPublisher()
  {
    subscription_.reset();
    publisher_.reset();
    delete access_;
  }

  bool isPublished()
  {
    std::lock_guard<mutex_t> guard(*getMutex());
    return is_published_;
  }

  mutex_t * getMutex()
  {
    return access_;
  }

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr)
  {
    is_published_ = true;
  }

  bool is_published_;

  mutex_t * access_;
};  // MaskPublisher

struct Point
{
  unsigned int x, y;
};

class TestNode : public ::testing::Test
{
public:
  TestNode() {}

  ~TestNode() {}

protected:
  void createMaps(unsigned char master_value, int8_t mask_value);
  void publishMaps();
  void createKeepoutFilter();
  void reset();
  void testKeepoutFilter(unsigned char free_value, unsigned char keepout_value);

  std::shared_ptr<nav2_costmap_2d::KeepoutFilter> keepout_filter_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> master_grid_;

  std::vector<Point> keepout_points_;

private:
  void verifyMasterGrid(unsigned char free_value, unsigned char keepout_value);

  nav2_util::LifecycleNode::SharedPtr node_;

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> mask_;

  std::shared_ptr<InfoPublisher> info_publisher_;
  std::shared_ptr<MaskPublisher> mask_publisher_;
};

void TestNode::createMaps(unsigned char master_value, int8_t mask_value)
{
  // Make map and mask overlapping as follows:
  //
  //        map     (10,10)
  //         *---------*
  //   mask  |  (5,5)  |
  //    *----+----*    |
  //    *////|////*    |
  //    *////*----*----*
  //    */////////*
  //    *---------*
  // (-5,-5)

  const unsigned int width = 10;
  const unsigned int height = 10;
  const double resolution = 1.0;

  // Create master_grid_
  master_grid_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
    width, height, resolution, 0.0, 0.0, master_value);

  // Create mask_
  mask_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  mask_->info.resolution = resolution;
  mask_->info.width = width;
  mask_->info.height = height;
  mask_->info.origin.position.x = -5.0;
  mask_->info.origin.position.y = -5.0;
  mask_->info.origin.position.z = 0.0;
  mask_->info.origin.orientation.x = 0.0;
  mask_->info.origin.orientation.y = 0.0;
  mask_->info.origin.orientation.z = 0.0;
  mask_->info.origin.orientation.w = 1.0;
  mask_->data.resize(width * height, mask_value);
}

void TestNode::publishMaps()
{
  info_publisher_ = std::make_shared<InfoPublisher>();
  while (!info_publisher_->isPublished()) {
    rclcpp::spin_some(info_publisher_);
    std::this_thread::sleep_for(100ms);
  }

  mask_publisher_ = std::make_shared<MaskPublisher>(*mask_);
  while (!mask_publisher_->isPublished()) {
    rclcpp::spin_some(mask_publisher_);
    std::this_thread::sleep_for(100ms);
  }
}

void TestNode::reset()
{
  mask_.reset();
  master_grid_.reset();
  info_publisher_.reset();
  mask_publisher_.reset();
  keepout_filter_.reset();
  node_.reset();
  keepout_points_.clear();
}

void TestNode::verifyMasterGrid(unsigned char free_value, unsigned char keepout_value)
{
  unsigned int x, y;
  bool is_checked;

  for (y = 0; y < master_grid_->getSizeInCellsY(); y++) {
    for (x = 0; x < master_grid_->getSizeInCellsX(); x++) {
      is_checked = false;
      for (std::vector<Point>::iterator it = keepout_points_.begin();
        it != keepout_points_.end(); it++)
      {
        if (x == it->x && y == it->y) {
          EXPECT_EQ(master_grid_->getCost(x, y), keepout_value);
          is_checked = true;
          break;
        }
      }
      if (!is_checked) {
        EXPECT_EQ(master_grid_->getCost(x, y), free_value);
      }
    }
  }
}

void TestNode::createKeepoutFilter()
{
  node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  node_->declare_parameter(
    FILTER_NAME + ".filter_info_topic", rclcpp::ParameterValue(INFO_TOPIC));
  node_->set_parameter(
    rclcpp::Parameter(FILTER_NAME + ".filter_info_topic", INFO_TOPIC));

  keepout_filter_ = std::make_shared<nav2_costmap_2d::KeepoutFilter>();
  keepout_filter_->initialize(&layers, FILTER_NAME, &tf, node_, nullptr, nullptr);
  keepout_filter_->initializeFilter(INFO_TOPIC);

  // Wait until mask will be received by KeepoutFilter
  while (!keepout_filter_->isActive()) {
    rclcpp::spin_some(node_->get_node_base_interface());
    std::this_thread::sleep_for(100ms);
  }
}

void TestNode::testKeepoutFilter(unsigned char free_value, unsigned char keepout_value)
{
  geometry_msgs::msg::Pose2D pose;
  // Intersection window: added 4 points
  keepout_filter_->process(*master_grid_, 1, 3, 3, 6, pose);
  keepout_points_.push_back(Point{1, 3});
  keepout_points_.push_back(Point{2, 3});
  keepout_points_.push_back(Point{1, 4});
  keepout_points_.push_back(Point{2, 4});
  verifyMasterGrid(free_value, keepout_value);
  // Two windows outside on the horisontal/vertical edge: no new points added
  keepout_filter_->process(*master_grid_, 1, 5, 3, 6, pose);
  keepout_filter_->process(*master_grid_, 5, 1, 6, 3, pose);
  verifyMasterGrid(free_value, keepout_value);
  // Corner window: added 1 point
  keepout_filter_->process(*master_grid_, 4, 4, 5, 5, pose);
  keepout_points_.push_back(Point{4, 4});
  verifyMasterGrid(free_value, keepout_value);
  // Outside window: no new points added
  keepout_filter_->process(*master_grid_, 7, 7, 9, 9, pose);
  verifyMasterGrid(free_value, keepout_value);
}

TEST_F(TestNode, testFreeMasterLethalKeepout)
{
  // Initilize test system
  createMaps(nav2_costmap_2d::FREE_SPACE, nav2_util::OCC_GRID_OCCUPIED);
  publishMaps();
  createKeepoutFilter();

  // Test KeepoutFilter
  testKeepoutFilter(nav2_costmap_2d::FREE_SPACE, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testUnknownMasterNonLethalKeepout)
{
  // Initilize test system
  createMaps(
    nav2_costmap_2d::NO_INFORMATION,
    (nav2_util::OCC_GRID_OCCUPIED - nav2_util::OCC_GRID_FREE) / 2);
  publishMaps();
  createKeepoutFilter();

  // Test KeepoutFilter
  testKeepoutFilter(
    nav2_costmap_2d::NO_INFORMATION,
    (nav2_costmap_2d::LETHAL_OBSTACLE - nav2_costmap_2d::FREE_SPACE) / 2);

  // Clean-up
  keepout_filter_->resetFilter();
  reset();
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
