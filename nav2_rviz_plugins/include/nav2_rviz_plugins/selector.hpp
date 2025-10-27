// Copyright (c) 2024 Neobotix GmbH
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

#ifndef NAV2_RVIZ_PLUGINS__SELECTOR_HPP_
#define NAV2_RVIZ_PLUGINS__SELECTOR_HPP_

#include <QtWidgets>
#include <QFrame>
#include <QGridLayout>
#include <QScrollArea>
#include <QToolButton>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "vector"
#include "memory"
#include "string"
#include "std_msgs/msg/string.hpp"

class QPushButton;

namespace nav2_rviz_plugins
{
class Selector : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Selector(QWidget * parent = 0);
  ~Selector();

private:
  void loadPlugins();

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_controller_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_planner_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_goal_checker_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_smoother_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_progress_checker_;

  bool plugins_loaded_ = false;
  bool server_failed_ = false;

  std::thread load_plugins_thread_;

  QVBoxLayout * main_layout_;
  QHBoxLayout * row_1_layout_;
  QHBoxLayout * row_2_layout_;
  QHBoxLayout * row_3_layout_;
  QHBoxLayout * row_1_label_layout_;
  QHBoxLayout * row_2_label_layout_;
  QHBoxLayout * row_3_label_layout_;
  QComboBox * controller_;
  QComboBox * planner_;
  QComboBox * goal_checker_;
  QComboBox * smoother_;
  QComboBox * progress_checker_;

  void setController();
  void setPlanner();
  void setGoalChecker();
  void setSmoother();
  void setProgressChecker();

  /*
    * @brief Set the selection from the combo box
    * @param combo_box The combo box to set the selection for
    * @param publisher Publish the selected plugin
  */
  void setSelection(
    QComboBox * combo_box,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher);

protected:
  QVBoxLayout * layout1 = new QVBoxLayout;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__SELECTOR_HPP_
