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

#ifndef NAV2_RVIZ_PLUGINS__ROS_ACTION_QEVENT_HPP_
#define NAV2_RVIZ_PLUGINS__ROS_ACTION_QEVENT_HPP_

#include <QAbstractTransition>

namespace nav2_rviz_plugins
{

enum class QActionState
{
  ACTIVE,
  INACTIVE
};

/// Custom Event to track state of ROS Action
struct ROSActionQEvent : public QEvent
{
  explicit ROSActionQEvent(QActionState state)
  : QEvent(QEvent::Type(QEvent::User + 1)),
    state_(state) {}

  QActionState state_;
};

/// Custom Transition to check whether ROS Action state has changed
class ROSActionQTransition : public QAbstractTransition
{
public:
  explicit ROSActionQTransition(QActionState initial_status)
  : status_(initial_status)
  {}

  ~ROSActionQTransition() {}

protected:
  virtual bool eventTest(QEvent * e)
  {
    if (e->type() != QEvent::Type(QEvent::User + 1)) {  // ROSActionEvent
      return false;
    }
    ROSActionQEvent * action_event = static_cast<ROSActionQEvent *>(e);
    return status_ != action_event->state_;
  }

  virtual void onTransition(QEvent *) {}
  QActionState status_;
};

}  // namespace nav2_rviz_plugins

#endif  //  NAV2_RVIZ_PLUGINS__ROS_ACTION_QEVENT_HPP_
