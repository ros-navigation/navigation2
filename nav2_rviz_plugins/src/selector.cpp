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

#include "nav2_rviz_plugins/selector.hpp"
#include "rviz_common/display_context.hpp"

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{
Selector::Selector(QWidget * parent)
: Panel(parent)
{
  client_node_ = std::make_shared<rclcpp::Node>("nav2_rviz_selector_node");
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  pub_controller_ =
    client_node_->create_publisher<std_msgs::msg::String>("controller_selector", qos);
  pub_planner_ = client_node_->create_publisher<std_msgs::msg::String>("planner_selector", qos);
  pub_goal_checker_ =
    client_node_->create_publisher<std_msgs::msg::String>("goal_checker_selector", qos);
  pub_smoother_ = client_node_->create_publisher<std_msgs::msg::String>("smoother_selector", qos);
  pub_progress_checker_ =
    client_node_->create_publisher<std_msgs::msg::String>("progress_checker_selector", qos);

  main_layout_ = new QVBoxLayout;
  row_1_label_layout_ = new QHBoxLayout;
  row_2_label_layout_ = new QHBoxLayout;
  row_3_label_layout_ = new QHBoxLayout;
  row_1_layout_ = new QHBoxLayout;
  row_2_layout_ = new QHBoxLayout;
  row_3_layout_ = new QHBoxLayout;
  controller_ = new QComboBox;
  planner_ = new QComboBox;
  goal_checker_ = new QComboBox;
  smoother_ = new QComboBox;
  progress_checker_ = new QComboBox;

  main_layout_->setContentsMargins(10, 10, 10, 10);

  row_1_label_layout_->addWidget(new QLabel("Controller"));
  row_1_layout_->addWidget(controller_);
  row_1_label_layout_->addWidget(new QLabel("Planner"));
  row_1_layout_->addWidget(planner_);
  row_2_label_layout_->addWidget(new QLabel("Goal Checker"));
  row_2_layout_->addWidget(goal_checker_);
  row_2_label_layout_->addWidget(new QLabel("Smoother"));
  row_2_layout_->addWidget(smoother_);
  row_3_label_layout_->addWidget(new QLabel("Progress Checker"));
  row_3_layout_->addWidget(progress_checker_);

  main_layout_->addLayout(row_1_label_layout_);
  main_layout_->addLayout(row_1_layout_);
  main_layout_->addLayout(row_2_label_layout_);
  main_layout_->addLayout(row_2_layout_);
  main_layout_->addLayout(row_3_label_layout_);
  main_layout_->addLayout(row_3_layout_);

  setLayout(main_layout_);
  timer_.start(200, this);

  connect(
    controller_, QOverload<int>::of(&QComboBox::activated), this,
    &Selector::setController);

  connect(
    planner_, QOverload<int>::of(&QComboBox::activated), this,
    &Selector::setPlanner);

  connect(
    goal_checker_, QOverload<int>::of(&QComboBox::activated), this,
    &Selector::setGoalChecker);

  connect(
    smoother_, QOverload<int>::of(&QComboBox::activated), this,
    &Selector::setSmoother);

  connect(
    progress_checker_, QOverload<int>::of(&QComboBox::activated), this,
    &Selector::setProgressChecker);
}

Selector::~Selector()
{
}

// Publish the selected controller or planner
void Selector::setSelection(
  QComboBox * combo_box, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
{
  // If "default" option is selected, it gets removed and the next item is selected
  if (combo_box->findText("Default") != -1) {
    combo_box->removeItem(0);
  }

  // if there are no plugins available, return
  if (combo_box->count() == 0) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = combo_box->currentText().toStdString();

  publisher->publish(msg);
  timer_.start(200, this);
}

// Call setSelection() for controller
void Selector::setController()
{
  setSelection(controller_, pub_controller_);
}

// Call setSelection() for planner
void Selector::setPlanner()
{
  setSelection(planner_, pub_planner_);
}

// Call setSelection() for goal checker
void Selector::setGoalChecker()
{
  setSelection(goal_checker_, pub_goal_checker_);
}

// Call setSelection() for smoother
void Selector::setSmoother()
{
  setSelection(smoother_, pub_smoother_);
}

void Selector::setProgressChecker()
{
  setSelection(progress_checker_, pub_progress_checker_);
}

// Load the available plugins into the combo box
void Selector::pluginLoader(
  rclcpp::Node::SharedPtr node,
  const std::string & server_name,
  const std::string & plugin_type,
  QComboBox * combo_box)
{
  auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(node, server_name);

  // Do not load the plugins if the combo box is already populated
  if (combo_box->count() > 0) {
    return;
  }

  // Wait for the service to be available before calling it
  bool server_unavailable = false;
  while (!parameter_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(
      node->get_logger(),
      (server_name + " service not available").c_str());
    server_unavailable = true;
    server_failed_ = true;
    break;
  }

  // Loading the plugins into the combo box
  if (!plugins_loaded_) {
    // If server unavaialble, let the combo box be empty
    if (server_unavailable) {
      return;
    }
    combo_box->addItem("Default");
    auto parameters = parameter_client->get_parameters({plugin_type});
    auto str_arr = parameters[0].as_string_array();
    for (auto str : str_arr) {
      combo_box->addItem(QString::fromStdString(str));
    }
    combo_box->setCurrentText("Default");
  }
}

void
Selector::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == timer_.timerId()) {
    if (!plugins_loaded_) {
      pluginLoader(client_node_, "controller_server", "controller_plugins", controller_);
      pluginLoader(client_node_, "planner_server", "planner_plugins", planner_);
      pluginLoader(client_node_, "controller_server", "goal_checker_plugins", goal_checker_);
      pluginLoader(client_node_, "smoother_server", "smoother_plugins", smoother_);
      pluginLoader(
        client_node_, "controller_server", "progress_checker_plugins",
        progress_checker_);

      plugins_loaded_ = true;
    }

    // Restart the timer if the one of the server fails
    if (server_failed_ && !tried_once_) {
      RCLCPP_INFO(client_node_->get_logger(), "Retrying to connect to the failed server.");
      server_failed_ = false;
      plugins_loaded_ = false;
      tried_once_ = true;
      timer_.start(200, this);
      return;
    }

    timer_.stop();
  }
}

}  // namespace nav2_rviz_plugins
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::Selector, rviz_common::Panel)
