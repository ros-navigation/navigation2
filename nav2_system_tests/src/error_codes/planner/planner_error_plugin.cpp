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

#include "planner_error_plugin.hpp"

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::UnknownErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::StartOccupiedErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::GoalOccupiedErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::StartOutsideMapErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::GoalOutsideMapErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::NoValidPathErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::TimedOutErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::TFErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::NoViapointsGivenErrorPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(nav2_system_tests::CancelledPlanner, nav2_core::GlobalPlanner)
