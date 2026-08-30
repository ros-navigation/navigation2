// Copyright (c) 2026 Karinca Robotics
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

#include "pluginlib/class_list_macros.hpp"

#include "opennav_docking/graceful_controller.hpp"

// This translation unit exists only to register GracefulController with pluginlib,
// and is built into a library of its own (graceful_controller_plugin) that nothing
// links against.
//
// Once opennav_following is migrated off opennav_docking::Controller
// this file can be moved back into graceful_controller.cpp.
PLUGINLIB_EXPORT_CLASS(opennav_docking::GracefulController, opennav_docking::ControllerBase)
