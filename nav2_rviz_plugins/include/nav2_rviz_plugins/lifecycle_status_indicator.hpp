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

#ifndef LIFECYCLE_STATUS_INDICATOR_HPP_
#define LIFECYCLE_STATUS_INDICATOR_HPP_

#include <QtWidgets>

#include "rviz_default_plugins/visibility_control.hpp"

namespace nav2_rviz_plugins
{

class LedStatusIndicator : public QWidget {
public:

  void paintEvent(QPaintEvent *e) {
    QWidget::paintEvent(e);
    QPainter p(this);
    p.setBrush(stateColor_);
    p.drawEllipse(QPoint(width()/2, height()/2), 5, 5);
  }
  void setColor(Qt::GlobalColor color) {
      stateColor_ = color;
  }

private:
  Qt::GlobalColor stateColor_ = Qt::gray;
};


class RVIZ_DEFAULT_PLUGINS_PUBLIC LifecycleStatusIndicator : public QWidget
{
  Q_OBJECT

public:

  LifecycleStatusIndicator(QString node_name) {

    current_state_ = "Inactive";

    QHBoxLayout *layout = new QHBoxLayout();

    led_status_ = new LedStatusIndicator();
    led_status_->setMinimumHeight(15);
    led_status_->setMinimumWidth(15);

    name_ = new QLabel(this);
    name_->setText(node_name + ":");
    name_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    status_ = new QLabel(this);
    status_->setText(current_state_);

    layout->addWidget(led_status_);
    layout->addWidget(name_);
    layout->addWidget(status_);
    setLayout(layout);

    this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }

public slots:
  void setInactive() {
    current_state_ = "Paused";
    status_->setText(current_state_);
    led_status_->setColor(Qt::GlobalColor::blue);
    this->update();
  }
  void setActive() {
    current_state_ = "Active";
    status_->setText(current_state_);
    led_status_->setColor(Qt::GlobalColor::green);
    this->update();
  }
  void setTimedOut() {
    if(current_state_ == "Paused")
        return;
    current_state_ = "Inactive";
    status_->setText(current_state_);
    led_status_->setColor(Qt::GlobalColor::gray);
    this->update();
  }

private:
  QLabel *name_, *status_;
  LedStatusIndicator *led_status_;
  QString current_state_;

};

}  // namespace nav2_rviz_plugins

#endif // NAV2_LIFECYCLE_STATUS_INDICATOR_HPP_
