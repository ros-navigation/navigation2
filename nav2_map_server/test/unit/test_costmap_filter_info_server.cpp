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
// limitations under the License.

#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <chrono>
#include <limits>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "nav2_map_server/costmap_filter_info_server.hpp"

using namespace std::chrono_literals;

typedef std::recursive_mutex mutex_t;

static const char FILTER_INFO_TOPIC[] = "filter_info";
static const int TYPE = 1;
static const char MASK_TOPIC[] = "mask";
static const double BASE = 0.1;
static const double MULTIPLIER = 0.2;

static const double EPSILON = std::numeric_limits<float>::epsilon();

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class InfoServerWrapper : public nav2_map_server::CostmapFilterInfoServer
{
public:
  void start()
  {
    on_configure(get_current_state());
    on_activate(get_current_state());
  }

  void stop()
  {
    on_deactivate(get_current_state());
    on_cleanup(get_current_state());
    on_shutdown(get_current_state());
  }

  void deactivate()
  {
    on_deactivate(get_current_state());
  }

  void activate()
  {
    on_activate(get_current_state());
  }
};

class InfoServerTester : public ::testing::Test
{
public:
  InfoServerTester()
  : info_server_(nullptr), info_(nullptr), subscription_(nullptr)
  {
    access_ = new mutex_t();

    info_server_ = std::make_shared<InfoServerWrapper>();
    try {
      info_server_->set_parameter(rclcpp::Parameter("filter_info_topic", FILTER_INFO_TOPIC));
      info_server_->set_parameter(rclcpp::Parameter("type", TYPE));
      info_server_->set_parameter(rclcpp::Parameter("mask_topic", MASK_TOPIC));
      info_server_->set_parameter(rclcpp::Parameter("base", BASE));
      info_server_->set_parameter(rclcpp::Parameter("multiplier", MULTIPLIER));
    } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
      RCLCPP_ERROR(
        info_server_->get_logger(),
        "Error while setting parameters for CostmapFilterInfoServer: %s", ex.what());
      throw;
    }

    info_server_->start();

    subscription_ = info_server_->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
      FILTER_INFO_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&InfoServerTester::infoCallback, this, std::placeholders::_1));
  }

  ~InfoServerTester()
  {
    info_server_->stop();
    info_server_.reset();
    subscription_.reset();
  }

  bool isReceived()
  {
    std::lock_guard<mutex_t> guard(*getMutex());
    if (info_) {
      return true;
    } else {
      return false;
    }
  }

  mutex_t * getMutex()
  {
    return access_;
  }

protected:
  std::shared_ptr<InfoServerWrapper> info_server_;
  nav2_msgs::msg::CostmapFilterInfo::SharedPtr info_;

private:
  void infoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
  {
    std::lock_guard<mutex_t> guard(*getMutex());
    info_ = msg;
  }

  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr subscription_;

  mutex_t * access_;
};

TEST_F(InfoServerTester, testCostmapFilterInfoPublish)
{
  rclcpp::Time start_time = info_server_->now();
  while (!isReceived()) {
    rclcpp::spin_some(info_server_->get_node_base_interface());
    std::this_thread::sleep_for(100ms);
    // Waiting no more than 5 seconds
    ASSERT_TRUE((info_server_->now() - start_time) <= rclcpp::Duration(5000ms));
  }

  // Checking received CostmapFilterInfo for consistency
  EXPECT_EQ(info_->type, TYPE);
  EXPECT_EQ(info_->filter_mask_topic, MASK_TOPIC);
  EXPECT_NEAR(info_->base, BASE, EPSILON);
  EXPECT_NEAR(info_->multiplier, MULTIPLIER, EPSILON);
}

TEST_F(InfoServerTester, testCostmapFilterInfoDeactivateActivate)
{
  info_server_->deactivate();
  info_ = nullptr;
  info_server_->activate();

  rclcpp::Time start_time = info_server_->now();
  while (!isReceived()) {
    rclcpp::spin_some(info_server_->get_node_base_interface());
    std::this_thread::sleep_for(100ms);
    // Waiting no more than 5 seconds
    ASSERT_TRUE((info_server_->now() - start_time) <= rclcpp::Duration(5000ms));
  }

  // Checking received CostmapFilterInfo for consistency
  EXPECT_EQ(info_->type, TYPE);
  EXPECT_EQ(info_->filter_mask_topic, MASK_TOPIC);
  EXPECT_NEAR(info_->base, BASE, EPSILON);
  EXPECT_NEAR(info_->multiplier, MULTIPLIER, EPSILON);
}
