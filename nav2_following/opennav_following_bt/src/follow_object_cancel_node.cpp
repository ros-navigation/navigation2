// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#include <string>
#include <memory>

#include "opennav_following_bt/follow_object_cancel_node.hpp"

namespace opennav_following_bt
{

FollowObjectCancel::FollowObjectCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtCancelActionNode<opennav_following_msgs::action::FollowObject>(
    xml_tag_name, action_name, conf)
{
}

}  // namespace opennav_following_bt

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<opennav_following_bt::FollowObjectCancel>(
        name, "follow_object", config);
    };

  factory.registerBuilder<opennav_following_bt::FollowObjectCancel>(
    "CancelFollowObject", builder);
}
