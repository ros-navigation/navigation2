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

#ifndef NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_

#include <QtWidgets>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "rviz_common/panel.hpp"

class QPushButton;

namespace nav2_rviz_plugins
{

/// Panel to interface to the nav2 stack
class Nav2Panel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Nav2Panel(QWidget * parent = 0);
  virtual ~Nav2Panel() {}

  void onInitialize() override;

  /// Load and save configuration data
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onStartup();
  void onShutdown();

private:
  void loadLogFiles();

  // The client used to control the nav2 stack
  nav2_lifecycle_manager::LifecycleManagerClient client_;

  QPushButton * start_stop_button_{nullptr};

  QStateMachine machine_;

  QState * initial_{nullptr};
  QState * starting_{nullptr};
  QState * stopping_{nullptr};
};

}  // namespace nav2_rviz_plugins

#endif  //  NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
