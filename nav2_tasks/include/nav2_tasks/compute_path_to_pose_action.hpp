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

#ifndef NAV2_TASKS__COMPUTE_PATH_TO_POSE_ACTION_HPP_
#define NAV2_TASKS__COMPUTE_PATH_TO_POSE_ACTION_HPP_

#include <string>
#include <memory>
#include "nav2_tasks/bt_conversions.hpp"
#include "nav2_tasks/bt_action_node.hpp"
#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace nav2_tasks
{

class ComputePathToPoseAction
  : public BtActionNode<ComputePathToPoseCommand, ComputePathToPoseResult>
{
public:
  explicit ComputePathToPoseAction(const std::string & action_name)
  : BtActionNode<ComputePathToPoseCommand, ComputePathToPoseResult>(action_name)
  {
  }

  void onInit() override
  {
    printf("ComputePathToPoseBTAction: onInit\n");
    command_ =
      blackboard()->template get<nav2_tasks::ComputePathToPoseCommand::SharedPtr>("endpoints");

    result_ = blackboard()->template get<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path");
    result2 = blackboard()->template get<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path");
    printf("ComputePathToPoseBTAction: path: %p\n", (void *) result_.get());
  }

  void onSuccess() override
  {
    //result_ = blackboard()->template get<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path");
    printf("ComputePathToPoseAction: onSuccess\n");
    int index = 0;
    for (auto pose : result_->poses) {
      printf("point %u x: %0.2f, y: %0.2f\n", index, pose.position.x, pose.position.y);
      index++;
    }

    //auto result2 = blackboard()->template get<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path");
    index = 0;
    printf("ComputePathToPoseAction: onSuccess2\n");
    for (auto pose : result2->poses) {
      printf("point %u x: %0.2f, y: %0.2f\n", index, pose.position.x, pose.position.y);
      index++;
    }
  }

  nav2_tasks::ComputePathToPoseResult::SharedPtr result2;

};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__COMPUTE_PATH_TO_POSE_ACTION_HPP_
