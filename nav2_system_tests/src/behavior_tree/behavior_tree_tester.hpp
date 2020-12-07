// Copyright (c) 2020 Vinny Ruia
// Copyright (c) 2020 Sarthak Mittal
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
// limitations under the License. Reserved.

#ifndef BEHAVIOR_TREE__BEHAVIOR_TREE_TESTER_HPP_
#define BEHAVIOR_TREE__BEHAVIOR_TREE_TESTER_HPP_

#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

// todo need to fix CMakeLists so that we get the orginal headerss
#include "test_action_server.hpp"
#include "test_service.hpp"
// #include "nav2_behavior_tree/test/test_action_server.hpp"
// #include "nav2_behavior_tree/test/test_service.hpp"

#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"
#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"
#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"
#include "nav2_behavior_tree/plugins/action/spin_action.hpp"
#include "nav2_behavior_tree/plugins/action/wait_action.hpp"
#include "nav2_behavior_tree/plugins/action/back_up_action.hpp"

namespace nav2_system_tests
{
template<class ActionT>
class FakeActionServer : public TestActionServer<ActionT>
{
public:
  FakeActionServer()
  : TestActionServer<ActionT>("fake_action_server")
  {
  }

protected:
  void execute(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>
    goal_handle)
  override
  {
    auto result = std::make_shared<typename ActionT::Result>();
    bool return_success = getReturnSuccess();
    if (return_success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
  bool getReturnSuccess();
};

class ClearEntireCostmapService : public TestService<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  ClearEntireCostmapService()
  : TestService("clear_entire_costmap")
  {
  }
};

class ComputePathToPoseActionServer : public FakeActionServer<nav2_msgs::action::ComputePathToPose>
{
public:
  ComputePathToPoseActionServer()
  : FakeActionServer()
  {
  }
};

class FollowPathActionServer : public FakeActionServer<nav2_msgs::action::FollowPath>
{
public:
  FollowPathActionServer()
  : FakeActionServer()
  {
  }
};

class SpinActionServer : public FakeActionServer<nav2_msgs::action::Spin>
{
public:
  SpinActionServer()
  : FakeActionServer()
  {
  }
};

class WaitActionServer : public FakeActionServer<nav2_msgs::action::Wait>
{
public:
  WaitActionServer()
  : FakeActionServer()
  {
  }
};

class BackUpActionServer : public FakeActionServer<nav2_msgs::action::BackUp>
{
public:
  BackUpActionServer()
  : FakeActionServer()
  {
  }
};

struct should_action_server_return_success_t
{
  bool compute_path_to_pose = true;
  bool follow_path = true;
  bool wait = true;
  bool back_up = true;
  bool spin = true;
};

class BehaviorTreeTester
{
public:
  BehaviorTreeTester();
  ~BehaviorTreeTester();

  void activate();

  void deactivate();

  bool isActive() const
  {
    return is_active_;
  }

  bool defaultBehaviorTreeTest(
    struct should_action_server_return_success_t test_case);

private:
  bool is_active_;
  rclcpp::Node::SharedPtr node_;
};

}  //  namespace nav2_system_tests
#endif  //  BEHAVIOR_TREE__BEHAVIOR_TREE_TESTER_HPP_
