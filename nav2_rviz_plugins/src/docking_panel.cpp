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

#include "nav2_util/geometry_utils.hpp"
#include "nav2_rviz_plugins/docking_panel.hpp"
#include "nav2_rviz_plugins/ros_action_qevent.hpp"
#include "nav2_rviz_plugins/utils.hpp"

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{

DockingPanel::DockingPanel(QWidget * parent)
: Panel(parent),
  server_timeout_(100)
{
  // Create the control buttons and its tooltip
  main_layout_ = new QVBoxLayout;
  info_layout_ = new QHBoxLayout;
  feedback_layout_ = new QVBoxLayout;
  dock_id_layout_ = new QHBoxLayout;
  dock_type_layout_ = new QHBoxLayout;
  dock_pose_layout_ = new QHBoxLayout;
  nav_stage_layout_ = new QHBoxLayout;
  dock_type_ = new QComboBox;
  docking_button_ = new QPushButton;
  undocking_button_ = new QPushButton;
  docking_goal_status_indicator_ = new QLabel;
  docking_feedback_indicator_ = new QLabel;
  docking_result_indicator_ = new QLabel;
  use_dock_id_checkbox_ = new QCheckBox;
  nav_to_staging_checkbox_ = new QCheckBox;
  dock_id_ = new QLineEdit;
  dock_pose_x_ = new QLineEdit;
  dock_pose_y_ = new QLineEdit;
  dock_pose_yaw_ = new QLineEdit;

  // Create the state machine used to present the proper control button states in the UI
  const char * nav_to_stage_msg = "Navigate to the staging pose before docking";
  const char * use_dock_id_msg = "Use the dock id or the dock pose to dock the robot";
  const char * dock_msg = "Dock the robot at the specified docking station";
  const char * undock_msg = "Undock the robot from the docking station";
  const char * cancel_dock_msg = "Cancel the current docking action";
  const char * cancel_undock_msg = "Cancel the current undocking action";

  docking_goal_status_indicator_->setText(nav2_rviz_plugins::getGoalStatusLabel());
  docking_feedback_indicator_->setText(getDockFeedbackLabel());
  docking_goal_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  docking_feedback_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  pre_initial_ = new QState();
  pre_initial_->setObjectName("pre_initial");
  pre_initial_->assignProperty(docking_button_, "text", "Dock robot");
  pre_initial_->assignProperty(docking_button_, "enabled", false);

  pre_initial_->assignProperty(undocking_button_, "text", "Undock robot");
  pre_initial_->assignProperty(undocking_button_, "enabled", false);

  pre_initial_->assignProperty(nav_to_staging_checkbox_, "enabled", false);
  pre_initial_->assignProperty(nav_to_staging_checkbox_, "checked", true);
  pre_initial_->assignProperty(use_dock_id_checkbox_, "enabled", false);
  pre_initial_->assignProperty(use_dock_id_checkbox_, "checked", true);
  pre_initial_->assignProperty(dock_id_, "enabled", false);
  pre_initial_->assignProperty(dock_type_, "enabled", false);
  pre_initial_->assignProperty(dock_pose_x_, "enabled", false);
  pre_initial_->assignProperty(dock_pose_y_, "enabled", false);
  pre_initial_->assignProperty(dock_pose_yaw_, "enabled", false);

  // State entered when the docking / undocking action is not active
  idle_ = new QState();
  idle_->setObjectName("idle");
  idle_->assignProperty(docking_button_, "text", "Dock robot");
  idle_->assignProperty(docking_button_, "toolTip", dock_msg);
  idle_->assignProperty(docking_button_, "enabled", true);

  idle_->assignProperty(undocking_button_, "text", "Undock robot");
  idle_->assignProperty(undocking_button_, "toolTip", undock_msg);
  idle_->assignProperty(undocking_button_, "enabled", true);

  idle_->assignProperty(nav_to_staging_checkbox_, "enabled", true);
  idle_->assignProperty(nav_to_staging_checkbox_, "toolTip", nav_to_stage_msg);
  idle_->assignProperty(use_dock_id_checkbox_, "enabled", true);
  idle_->assignProperty(use_dock_id_checkbox_, "toolTip", use_dock_id_msg);
  idle_->assignProperty(dock_id_, "enabled", true);
  idle_->assignProperty(dock_type_, "enabled", true);

  // State entered to cancel the docking action
  canceled_docking_ = new QState();
  canceled_docking_->setObjectName("canceled_docking");

  // State entered to cancel the undocking action
  canceled_undocking_ = new QState();
  canceled_undocking_->setObjectName("canceled_undocking");

  // State entered while the docking action is active
  docking_ = new QState();
  docking_->setObjectName("docking");
  docking_->assignProperty(docking_button_, "text", "Cancel docking");
  docking_->assignProperty(docking_button_, "toolTip", cancel_dock_msg);

  docking_->assignProperty(undocking_button_, "enabled", false);

  // State entered while the undocking action is active
  undocking_ = new QState();
  undocking_->setObjectName("undocking");
  undocking_->assignProperty(docking_button_, "enabled", false);

  undocking_->assignProperty(undocking_button_, "text", "Cancel undocking");
  undocking_->assignProperty(undocking_button_, "toolTip", cancel_undock_msg);

  QObject::connect(docking_, SIGNAL(entered()), this, SLOT(onDockingButtonPressed()));
  QObject::connect(undocking_, SIGNAL(entered()), this, SLOT(onUndockingButtonPressed()));
  QObject::connect(canceled_docking_, SIGNAL(exited()), this, SLOT(onCancelDocking()));
  QObject::connect(canceled_undocking_, SIGNAL(exited()), this, SLOT(onCancelUndocking()));

  // Start/Cancel button click transitions
  idle_->addTransition(docking_button_, SIGNAL(clicked()), docking_);
  idle_->addTransition(undocking_button_, SIGNAL(clicked()), undocking_);
  docking_->addTransition(docking_button_, SIGNAL(clicked()), canceled_docking_);
  undocking_->addTransition(undocking_button_, SIGNAL(clicked()), canceled_undocking_);

  // Internal state transitions
  canceled_docking_->addTransition(canceled_docking_, SIGNAL(entered()), idle_);
  canceled_undocking_->addTransition(canceled_undocking_, SIGNAL(entered()), idle_);

  // ROSAction Transitions: So when actions are updated remotely (failing, succeeding, etc)
  // the state of the application will also update. This means that if in the processing
  // states and then goes inactive, move back to the idle state. Vise versa as well.
  ROSActionQTransition * idleDockTransition = new ROSActionQTransition(QActionState::INACTIVE);
  idleDockTransition->setTargetState(docking_);
  idle_->addTransition(idleDockTransition);

  ROSActionQTransition * idleUndockTransition = new ROSActionQTransition(QActionState::INACTIVE);
  idleUndockTransition->setTargetState(undocking_);
  idle_->addTransition(idleUndockTransition);

  ROSActionQTransition * dockingTransition = new ROSActionQTransition(QActionState::ACTIVE);
  dockingTransition->setTargetState(idle_);
  docking_->addTransition(dockingTransition);

  ROSActionQTransition * undockingTransition = new ROSActionQTransition(QActionState::ACTIVE);
  undockingTransition->setTargetState(idle_);
  undocking_->addTransition(undockingTransition);

  client_node_ = std::make_shared<rclcpp::Node>("nav2_rviz_docking_panel_node");

  state_machine_.addState(pre_initial_);
  state_machine_.addState(idle_);
  state_machine_.addState(docking_);
  state_machine_.addState(undocking_);
  state_machine_.addState(canceled_docking_);
  state_machine_.addState(canceled_undocking_);

  state_machine_.setInitialState(pre_initial_);

  // Delay starting initial thread until state machine has started or a race occurs
  QObject::connect(&state_machine_, SIGNAL(started()), this, SLOT(startThread()));
  state_machine_.start();

  // Create the layout for the panel
  info_layout_->addWidget(docking_goal_status_indicator_);
  info_layout_->addWidget(docking_result_indicator_);
  feedback_layout_->addWidget(docking_feedback_indicator_);

  QLabel * nav_stage_label = new QLabel("Nav. to staging pose");
  QLabel * dock_id_label = new QLabel("Dock id");
  QLabel * dock_type_label = new QLabel("Dock type");

  nav_stage_label->setFixedWidth(150);
  dock_id_label->setFixedWidth(150);
  dock_type_label->setFixedWidth(170);

  nav_stage_layout_->addWidget(nav_stage_label);
  nav_stage_layout_->addWidget(nav_to_staging_checkbox_);
  dock_id_layout_->addWidget(dock_id_label);
  dock_id_layout_->addWidget(use_dock_id_checkbox_);
  dock_id_layout_->addWidget(dock_id_);
  dock_type_layout_->addWidget(dock_type_label);
  dock_type_layout_->addWidget(dock_type_);
  dock_pose_layout_->addWidget(new QLabel("Dock pose {X"));
  dock_pose_layout_->addWidget(dock_pose_x_);
  dock_pose_layout_->addWidget(new QLabel("Y"));
  dock_pose_layout_->addWidget(dock_pose_y_);
  dock_pose_layout_->addWidget(new QLabel("θ"));
  dock_pose_layout_->addWidget(dock_pose_yaw_);
  dock_pose_layout_->addWidget(new QLabel("}"));

  QGroupBox * group_box = new QGroupBox();
  QVBoxLayout * group_box_layout = new QVBoxLayout;
  group_box_layout->addLayout(nav_stage_layout_);
  group_box_layout->addLayout(dock_id_layout_);
  group_box_layout->addLayout(dock_type_layout_);
  group_box_layout->addLayout(dock_pose_layout_);
  group_box->setLayout(group_box_layout);

  main_layout_->setContentsMargins(10, 10, 10, 10);
  main_layout_->addLayout(info_layout_);
  main_layout_->addLayout(feedback_layout_);
  main_layout_->addWidget(group_box);
  main_layout_->addWidget(docking_button_);
  main_layout_->addWidget(undocking_button_);

  setLayout(main_layout_);
  action_timer_.start(200, this);

  dock_client_ = rclcpp_action::create_client<Dock>(client_node_, "dock_robot");
  undock_client_ = rclcpp_action::create_client<Undock>(client_node_, "undock_robot");
  initial_thread_ = new InitialDockThread(dock_client_, undock_client_);
  connect(initial_thread_, &InitialDockThread::finished, initial_thread_, &QObject::deleteLater);

  QSignalTransition * activeDockSignal = new QSignalTransition(
    initial_thread_, &InitialDockThread::dockingActive);
  activeDockSignal->setTargetState(idle_);
  pre_initial_->addTransition(activeDockSignal);

  QSignalTransition * activeUndockSignal = new QSignalTransition(
    initial_thread_, &InitialDockThread::undockingActive);
  activeUndockSignal->setTargetState(idle_);
  pre_initial_->addTransition(activeUndockSignal);

  QObject::connect(
    initial_thread_, &InitialDockThread::dockingActive,
    [this] {
      // Load the plugins if not already loaded
      if (!plugins_loaded_) {
        RCLCPP_INFO(client_node_->get_logger(), "Loading dock plugins");
        nav2_rviz_plugins::pluginLoader(
        client_node_, server_failed_, "docking_server", "dock_plugins", dock_type_);
        plugins_loaded_ = true;
      }
    });

  // Conect buttons with functions
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
    });

  // Create action goal status subscribers
  docking_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "dock_robot/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      docking_goal_status_indicator_->setText(
        nav2_rviz_plugins::getGoalStatusLabel("Feedback", msg->status_list.back().status));
      // Reset values when action is completed
      if (msg->status_list.back().status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        docking_feedback_indicator_->setText(getDockFeedbackLabel());
      }
    });

  undocking_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "undock_robot/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      docking_goal_status_indicator_->setText(
        nav2_rviz_plugins::getGoalStatusLabel("Feedback", msg->status_list.back().status));
    });
}

void DockingPanel::startThread()
{
  // start initial thread now that state machine is started
  initial_thread_->start();
}

DockingPanel::~DockingPanel()
{
}

void DockingPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

void DockingPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void DockingPanel::onDockingButtonPressed()
{
  auto is_action_server_ready =
    dock_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(client_node_->get_logger(), "dock_robot action server is not available.");
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
  auto goal_msg = Dock::Goal();
  goal_msg.use_dock_id = use_dock_id_;
  goal_msg.navigate_to_staging_pose = nav_to_staging_checkbox_->isChecked();
  if (use_dock_id_) {
    if (dock_id_->text().isEmpty()) {
      RCLCPP_ERROR(client_node_->get_logger(), "Dock id is empty.");
      return;
    }
    goal_msg.dock_id = dock_id_->text().toStdString();

    RCLCPP_INFO(
      client_node_->get_logger(), "DockRobot will be called using dock id: %s",
      goal_msg.dock_id.c_str());

  } else {
    if (dock_pose_x_->text().isEmpty() || dock_pose_y_->text().isEmpty() ||
      dock_pose_yaw_->text().isEmpty())
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Dock pose is empty.");
      return;
    }
    goal_msg.dock_pose.header.frame_id = "map";
    goal_msg.dock_pose.header.stamp = client_node_->now();
    goal_msg.dock_pose.pose.position.x = dock_pose_x_->text().toDouble();
    goal_msg.dock_pose.pose.position.y = dock_pose_y_->text().toDouble();
    goal_msg.dock_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
      dock_pose_yaw_->text().toDouble());
    goal_msg.dock_type = combo_box->currentText().toStdString();

    RCLCPP_INFO(
      client_node_->get_logger(), "DockRobot will be called using dock pose: (%f, %f) and type: %s",
      goal_msg.dock_pose.pose.position.x, goal_msg.dock_pose.pose.position.y,
      goal_msg.dock_type.c_str());
  }

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

  action_timer_.start(200, this);
}

void DockingPanel::onUndockingButtonPressed()
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
        docking_result_indicator_->setText("");
      } else {
        docking_result_indicator_->setText(
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

  action_timer_.start(200, this);
}

void DockingPanel::dockIdCheckbox()
{
  if (use_dock_id_checkbox_->isChecked()) {
    use_dock_id_ = true;
    dock_id_->setEnabled(true);
    dock_pose_x_->setEnabled(false);
    dock_pose_y_->setEnabled(false);
    dock_pose_yaw_->setEnabled(false);
  } else {
    use_dock_id_ = false;
    dock_id_->setEnabled(false);
    dock_pose_x_->setEnabled(true);
    dock_pose_y_->setEnabled(true);
    dock_pose_yaw_->setEnabled(true);
  }
}

void DockingPanel::onCancelDocking()
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

  action_timer_.stop();
}

void DockingPanel::onCancelUndocking()
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

  action_timer_.stop();
}

void DockingPanel::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == action_timer_.timerId()) {
    // Check the status of the action servers
    if (state_machine_.configuration().contains(docking_)) {
      if (!dock_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = dock_goal_handle_->get_status();

      // Check if the goal is still executing
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
      } else {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        action_timer_.stop();
      }
    } else if (state_machine_.configuration().contains(undocking_)) {
      if (!undock_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = undock_goal_handle_->get_status();

      // Check if the goal is still executing
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
      } else {
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        action_timer_.stop();
      }
    }
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
