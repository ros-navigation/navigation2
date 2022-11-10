// Copyright (c) 2022 Joshua Wallace
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


#include "global_planners/unknown_error_planner.hpp"
#include "global_planners/planner_plugins.hpp"

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::UnknownErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::StartOccupiedErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::GoalOccupiedErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::StartOutsideMapErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::GoalOutsideMapErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::NoValidPathErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::TimedOutErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::TFErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::NoViapointsGivenErrorPlanner, nav2_core::GlobalPlanner)
