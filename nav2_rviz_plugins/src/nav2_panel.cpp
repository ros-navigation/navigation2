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

#include "nav2_rviz_plugins/nav2_panel.hpp"

#include <dirent.h>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include "rviz_common/display_context.hpp"

namespace nav2_rviz_plugins
{

Nav2Panel::Nav2Panel(QWidget * parent)
: Panel(parent)
{
  // Create the control button and its tooltip

  start_stop_button_ = new QPushButton;

  // Create the state machine used to present the proper control button states in the UI

  const char * startup_msg = "Configure and activate all nav2 lifecycle nodes";
  const char * shutdown_msg = "Deactivate, cleanup, and shutdown all nav2 lifecycle nodes";

  initial_ = new QState();
  initial_->setObjectName("initial");
  initial_->assignProperty(start_stop_button_, "text", "Startup");
  initial_->assignProperty(start_stop_button_, "toolTip", startup_msg);

  starting_ = new QState();
  starting_->setObjectName("starting");
  starting_->assignProperty(start_stop_button_, "text", "Shutdown");
  starting_->assignProperty(start_stop_button_, "toolTip", shutdown_msg);

  stopping_ = new QState();
  stopping_->setObjectName("stopping");
  stopping_->assignProperty(start_stop_button_, "enabled", false);

  QObject::connect(starting_, SIGNAL(entered()), this, SLOT(onStartup()));
  QObject::connect(stopping_, SIGNAL(entered()), this, SLOT(onShutdown()));

  initial_->addTransition(start_stop_button_, SIGNAL(clicked()), starting_);
  starting_->addTransition(start_stop_button_, SIGNAL(clicked()), stopping_);

  machine_.addState(initial_);
  machine_.addState(starting_);
  machine_.addState(stopping_);

  machine_.setInitialState(initial_);
  machine_.start();

  // Lay out the items in the panel

  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(start_stop_button_);
  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);
}

void
Nav2Panel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void
Nav2Panel::onStartup()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_lifecycle_manager::LifecycleManagerClient::startup,
      &client_));
}

void
Nav2Panel::onShutdown()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_lifecycle_manager::LifecycleManagerClient::shutdown,
      &client_));
}

void
Nav2Panel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void
Nav2Panel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::Nav2Panel, rviz_common::Panel)
