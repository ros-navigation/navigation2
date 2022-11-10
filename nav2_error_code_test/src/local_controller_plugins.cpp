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

#include "local_controllers/unknown_error_controller.hpp"
#include "local_controllers/controller_plugins.hpp"

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::UnknownErrorController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::TFErrorController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::FailedToMakeProgressErrorController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::PatienceExceededErrorController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::InvalidPathErrorController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::NoValidControlErrorController, nav2_core::Controller)
