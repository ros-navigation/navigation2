// Copyright (c) 2021, Samsung Research America
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

#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_waypoint_follower/waypoint_follower.hpp"
#include "rclcpp/rclcpp.hpp"

class WPShim : public nav2_waypoint_follower::WaypointFollower
{
public:
  WPShim()
  : nav2_waypoint_follower::WaypointFollower(rclcpp::NodeOptions())
  {
  }

  void configure()
  {
    rclcpp_lifecycle::State state;
    this->on_configure(state);
  }

  void activate()
  {
    rclcpp_lifecycle::State state;
    this->on_activate(state);
  }

  void deactivate()
  {
    rclcpp_lifecycle::State state;
    this->on_deactivate(state);
  }

  void cleanup()
  {
    rclcpp_lifecycle::State state;
    this->on_cleanup(state);
  }
};

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(WPTest, test_dynamic_parameters)
{
  auto follower = std::make_shared<WPShim>();
  follower->configure();
  follower->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    follower->get_node_base_interface(), follower->get_node_topics_interface(),
    follower->get_node_graph_interface(),
    follower->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("loop_rate", 100),
      rclcpp::Parameter("stop_on_failure", false)});

  rclcpp::spin_until_future_complete(
    follower->get_node_base_interface(),
    results);

  EXPECT_EQ(follower->get_parameter("loop_rate").as_int(), 100);
  EXPECT_EQ(follower->get_parameter("stop_on_failure").as_bool(), false);
  follower->deactivate();
  follower->cleanup();
  follower.reset();
}
