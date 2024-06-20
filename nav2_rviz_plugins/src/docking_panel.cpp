// Copyright (c) 2024 Alberto J. Tudela Roldán
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

// C++
#include <stdio.h>

// QT
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <rviz_common/display_context.hpp>

#include "nav2_rviz_plugins/docking_panel.hpp"
#include "nav2_rviz_plugins/utils.hpp"

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{

DockingPanel::DockingPanel(QWidget * parent)
: Panel(parent),
  server_timeout_(100)
{
  client_node_ = std::make_shared<rclcpp::Node>("nav2_rviz_docking_panel_node");

  main_layout_ = new QVBoxLayout;
  dock_info_layout_ = new QHBoxLayout;
  undock_info_layout_ = new QHBoxLayout;
  feedback_layout_ = new QVBoxLayout;
  dock_id_layout_ = new QHBoxLayout;
  dock_type_layout_ = new QHBoxLayout;
  dock_type_ = new QComboBox;
  docking_button_ = new QPushButton("Dock robot");
  undocking_button_ = new QPushButton("Undock robot");
  docking_goal_status_indicator_ = new QLabel;
  undocking_goal_status_indicator_ = new QLabel;
  docking_feedback_indicator_ = new QLabel;
  docking_result_indicator_ = new QLabel;
  undocking_result_indicator_ = new QLabel;
  use_dock_id_checkbox_ = new QCheckBox;
  dock_id_ = new QLineEdit;

  docking_button_->setEnabled(false);
  undocking_button_->setEnabled(false);
  dock_id_->setEnabled(false);
  use_dock_id_checkbox_->setEnabled(false);
  docking_goal_status_indicator_->setText(nav2_rviz_plugins::getGoalStatusLabel("Docking"));
  undocking_goal_status_indicator_->setText(nav2_rviz_plugins::getGoalStatusLabel("Undocking"));
  docking_feedback_indicator_->setText(getDockFeedbackLabel());
  docking_goal_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  undocking_goal_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  docking_feedback_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  dock_info_layout_->addWidget(docking_goal_status_indicator_);
  dock_info_layout_->addWidget(docking_result_indicator_);
  undock_info_layout_->addWidget(undocking_goal_status_indicator_);
  undock_info_layout_->addWidget(undocking_result_indicator_);
  feedback_layout_->addWidget(docking_feedback_indicator_);
  dock_id_layout_->addWidget(new QLabel("Dock id"));
  dock_id_layout_->addWidget(use_dock_id_checkbox_);
  dock_id_layout_->addWidget(dock_id_);
  dock_type_layout_->addWidget(new QLabel("Dock type"));
  dock_type_layout_->addWidget(dock_type_);

  main_layout_->setContentsMargins(10, 10, 10, 10);
  main_layout_->addLayout(dock_info_layout_);
  main_layout_->addLayout(undock_info_layout_);
  main_layout_->addLayout(feedback_layout_);
  main_layout_->addLayout(dock_id_layout_);
  main_layout_->addLayout(dock_type_layout_);
  main_layout_->addWidget(docking_button_);
  main_layout_->addWidget(undocking_button_);

  setLayout(main_layout_);
  timer_.start(200, this);

  dock_client_ = rclcpp_action::create_client<Dock>(client_node_, "dock_robot");
  undock_client_ = rclcpp_action::create_client<Undock>(client_node_, "undock_robot");

  // Conect buttons with functions
  QObject::connect(docking_button_, SIGNAL(clicked()), this, SLOT(onDockingButtonPressed()));
  QObject::connect(undocking_button_, SIGNAL(clicked()), this, SLOT(onUndockingButtonPressed()));
  QObject::connect(
    use_dock_id_checkbox_, &QCheckBox::stateChanged, this, &DockingPanel::dockIdCheckbox);
}

void DockingPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Create action feedback subscriber
  docking_feedback_sub_ = node->create_subscription<Dock::Impl::FeedbackMessage>(
    "dock_robot/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const Dock::Impl::FeedbackMessage::SharedPtr msg) {
      docking_feedback_indicator_->setText(getDockFeedbackLabel(msg->feedback));
      docking_button_->setText("Cancel docking");
      undocking_button_->setEnabled(false);
      docking_in_progress_ = true;
    });

  undocking_feedback_sub_ = node->create_subscription<Undock::Impl::FeedbackMessage>(
    "undock_robot/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const Undock::Impl::FeedbackMessage::SharedPtr /*msg*/) {
      docking_button_->setEnabled(false);
      undocking_button_->setText("Cancel undocking");
      undocking_in_progress_ = true;
    });

  // Create action goal status subscribers
  docking_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "dock_robot/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      docking_goal_status_indicator_->setText(
        nav2_rviz_plugins::getGoalStatusLabel("Docking", msg->status_list.back().status));
      docking_button_->setText("Dock robot");
      docking_in_progress_ = false;
      // Reset values when action is completed
      if (msg->status_list.back().status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        docking_feedback_indicator_->setText(getDockFeedbackLabel());
      }
    });

  undocking_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "undock_robot/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      undocking_goal_status_indicator_->setText(
        nav2_rviz_plugins::getGoalStatusLabel("Undocking", msg->status_list.back().status));
      undocking_button_->setText("Undock robot");
      undocking_in_progress_ = false;
    });
}

DockingPanel::~DockingPanel()
{
}

void DockingPanel::onDockingButtonPressed()
{
  if (!docking_in_progress_) {
    startDocking();
  } else {
    cancelDocking();
  }
}

void DockingPanel::onUndockingButtonPressed()
{
  if (!undocking_in_progress_) {
    startUndocking();
  } else {
    cancelUndocking();
  }
}

void DockingPanel::startDocking()
{
  auto is_action_server_ready =
    dock_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(client_node_->get_logger(), "dock_robot action server is not available.");
    return;
  }

  if (use_dock_id_) {
    if (dock_id_->text().isEmpty()) {
      RCLCPP_ERROR(client_node_->get_logger(), "Dock id is empty.");
      return;
    }
  } else {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "You must check use_dock_id. Using dock pose is not currently supported.");
    return;
  }

  // Send the goal to the action server
  auto goal_msg = Dock::Goal();
  goal_msg.use_dock_id = use_dock_id_;
  goal_msg.dock_id = dock_id_->text().toStdString();

  RCLCPP_INFO(
    client_node_->get_logger(), "DockRobot will be called using dock id: %s",
    goal_msg.dock_id.c_str());

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
  send_goal_options.result_callback = [this](const DockGoalHandle::WrappedResult & result) {
      dock_goal_handle_.reset();
      if (result.result->success) {
        docking_result_indicator_->setText("");
      } else {
        docking_result_indicator_->setText(
          QString(std::string("(" + dockErrorToString(result.result->error_code) + ")").c_str()));
      }
    };

  auto future_goal_handle = dock_client_->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  dock_goal_handle_ = future_goal_handle.get();
  if (!dock_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }

  timer_.start(200, this);
}

void DockingPanel::startUndocking()
{
  auto is_action_server_ready =
    undock_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(client_node_->get_logger(), "undock_robot action server is not available.");
    return;
  }

  QComboBox * combo_box = dock_type_;
  // If "default" option is selected, it gets removed and the next item is selected
  if (combo_box->findText("Default") != -1) {
    combo_box->removeItem(0);
  }

  // If there are no plugins available, return
  if (combo_box->count() == 0) {
    return;
  }

  // Send the goal to the action server
  auto goal_msg = Undock::Goal();
  goal_msg.dock_type = combo_box->currentText().toStdString();

  RCLCPP_INFO(
    client_node_->get_logger(), "UndockRobot will be called using dock type: %s",
    goal_msg.dock_type.c_str());

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<Undock>::SendGoalOptions();
  send_goal_options.result_callback = [this](const UndockGoalHandle::WrappedResult & result) {
      undock_goal_handle_.reset();
      if (result.result->success) {
        undocking_result_indicator_->setText("");
      } else {
        undocking_result_indicator_->setText(
          QString(std::string("(" + dockErrorToString(result.result->error_code) + ")").c_str()));
      }
    };

  auto future_goal_handle = undock_client_->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  undock_goal_handle_ = future_goal_handle.get();
  if (!undock_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }

  timer_.start(200, this);
}

void DockingPanel::dockIdCheckbox()
{
  if (use_dock_id_checkbox_->isChecked()) {
    use_dock_id_ = true;
    dock_id_->setEnabled(true);
  } else {
    use_dock_id_ = false;
    dock_id_->setEnabled(false);
  }
}

void DockingPanel::cancelDocking()
{
  if (dock_goal_handle_) {
    auto future_cancel = dock_client_->async_cancel_goal(dock_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
    } else {
      dock_goal_handle_.reset();
    }
  }
}

void DockingPanel::cancelUndocking()
{
  if (undock_goal_handle_) {
    auto future_cancel = undock_client_->async_cancel_goal(undock_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
    } else {
      undock_goal_handle_.reset();
    }
  }
}

void DockingPanel::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == timer_.timerId()) {
    if (!plugins_loaded_) {
      nav2_rviz_plugins::pluginLoader(
        client_node_, server_failed_, "docking_server", "dock_plugins", dock_type_);
      plugins_loaded_ = true;
    }

    if (!server_failed_) {
      docking_button_->setEnabled(true);
      undocking_button_->setEnabled(true);
      use_dock_id_checkbox_->setEnabled(true);
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

inline QString DockingPanel::getDockFeedbackLabel(Dock::Feedback msg)
{
  return QString(std::string("<table>" + toLabel(msg) + "</table>").c_str());
}

template<typename T>
inline std::string DockingPanel::toLabel(T & msg)
{
  return std::string(
    "</td></tr><tr><td width=150>State:</td><td>" +
    dockStateToString(msg.state) +
    "</td></tr><tr><td width=150>Time taken:</td><td>" +
    toString(rclcpp::Duration(msg.docking_time).seconds(), 0) + " s"
    "</td></tr><tr><td width=150>Retries:</td><td>" +
    std::to_string(msg.num_retries) +
    "</td></tr>");
}

inline std::string DockingPanel::toString(double val, int precision)
{
  std::ostringstream out;
  out.precision(precision);
  out << std::fixed << val;
  return out.str();
}

inline std::string DockingPanel::dockStateToString(int16_t state)
{
  switch (state) {
    case 0:
      return "none";
    case 1:
      return "nav. to staging pose";
    case 2:
      return "initial perception";
    case 3:
      return "controlling";
    case 4:
      return "wait for charge";
    case 5:
      return "retry";
    default:
      return "none";
  }
}

inline std::string DockingPanel::dockErrorToString(int16_t error_code)
{
  switch (error_code) {
    case 0:
      return "none";
    case 901:
      return "dock not in database";
    case 902:
      return "dock not valid";
    case 903:
      return "failed to stage";
    case 904:
      return "failed to detect dock";
    case 905:
      return "failed to control";
    case 906:
      return "failed to charge";
    case 999:
    default:
      return "unknown";
  }
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::DockingPanel, rviz_common::Panel)
