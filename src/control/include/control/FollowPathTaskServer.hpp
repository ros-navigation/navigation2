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

#ifndef CONTROL__FOLLOWPATHTASKSERVER_HPP_
#define CONTROL__FOLLOWPATHTASKSERVER_HPP_

#include "task/TaskServer.hpp"
#include "nav2_msgs/msg/path.hpp"
#include "std_msgs/msg/empty.hpp"

using FollowPathCommand = nav2_msgs::msg::Path;
using FollowPathResult = std_msgs::msg::Empty;

typedef TaskServer<FollowPathCommand, FollowPathResult> FollowPathTaskServer;

#endif  // CONTROL__FOLLOWPATHTASKSERVER_HPP_
