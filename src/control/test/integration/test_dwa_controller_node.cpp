// Copyright (c) 2018 Intel Corporation
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

#include <control/FollowPathTaskClient.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

// rclcpp::init can only be called once per process, so this needs to be a global variable
class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    node = rclcpp::Node::make_shared("dwa_controller_test");
    client = std::make_unique<FollowPathTaskClient>("DwaController", node.get());
    while (node->count_subscribers("/DwaController_command") < 1) {
      rclcpp::spin_some(node);
    }
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<FollowPathTaskClient> client;
};

TEST_F(TestNode, ResultReturned)
{
  FollowPathCommand c;
  client->executeAsync(std::make_shared<FollowPathCommand>(c));
  FollowPathResult r;
  auto r_ptr = std::make_shared<FollowPathResult>(r);
  while (client->waitForResult(r_ptr, 1000) == TaskStatus::RUNNING) {
    rclcpp::spin_some(node);
  }
  SUCCEED();
}
