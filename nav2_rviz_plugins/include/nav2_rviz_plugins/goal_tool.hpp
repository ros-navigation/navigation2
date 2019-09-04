// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_RVIZ_PLUGINS__GOAL_TOOL_HPP_
#define NAV2_RVIZ_PLUGINS__GOAL_TOOL_HPP_

#include <QObject>

#include <memory>

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{

class DisplayContext;

namespace properties
{
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace nav2_rviz_plugins
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC GoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  GoalTool();
  ~GoalTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__GOAL_TOOL_HPP_
