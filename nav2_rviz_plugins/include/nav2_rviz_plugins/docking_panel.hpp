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

#ifndef NAV2_RVIZ_PLUGINS__DOCKING_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__DOCKING_PANEL_HPP_

// QT
#include <QtWidgets>
#include <QBasicTimer>

#include <string>

// ROS
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/undock_robot.hpp"

class QLineEdit;
class QPushButton;

namespace nav2_rviz_plugins
{

class InitialDockThread;

/// Panel to interface to the docking server
class DockingPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DockingPanel(QWidget * parent = 0);
  virtual ~DockingPanel();

  void onInitialize() override;

  /// Load and save configuration data
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void startThread();
  void onStartup();
  void onDockingButtonPressed();
  void onUndockingButtonPressed();
  void onCancelDocking();
  void onCancelUndocking();
  void dockIdCheckbox();

private:
  using Dock = nav2_msgs::action::DockRobot;
  using Undock = nav2_msgs::action::UndockRobot;
  using DockGoalHandle = rclcpp_action::ClientGoalHandle<Dock>;
  using UndockGoalHandle = rclcpp_action::ClientGoalHandle<Undock>;

  // The (non-spinning) client node used to invoke the action client
  void timerEvent(QTimerEvent * event) override;

  // Create label string from feedback msg
  static inline QString getDockFeedbackLabel(Dock::Feedback msg = Dock::Feedback());

  // Create label string from status msg
  template<typename T>
  static inline std::string toLabel(T & msg);

  // Round off double to the specified precision and convert to string
  static inline std::string toString(double val, int precision = 0);

  // Convert the dock state and error code to string
  static inline std::string dockStateToString(int16_t state);
  static inline std::string dockErrorToString(int16_t error_code);

  // The (non-spinning) client node used to invoke the action client
  rclcpp::Node::SharedPtr client_node_;

  // Timeout value when waiting for action servers to respond
  std::chrono::milliseconds server_timeout_;

  // A timer used to check on the completion status of the action
  QBasicTimer action_timer_;

  // The Dock and Undock action client
  rclcpp_action::Client<Dock>::SharedPtr dock_client_;
  rclcpp_action::Client<Undock>::SharedPtr undock_client_;

  // Docking / Undocking action feedback subscribers
  rclcpp::Subscription<Dock::Impl::FeedbackMessage>::SharedPtr docking_feedback_sub_;
  rclcpp::Subscription<Undock::Impl::FeedbackMessage>::SharedPtr undocking_feedback_sub_;
  rclcpp::Subscription<Dock::Impl::GoalStatusMessage>::SharedPtr docking_goal_status_sub_;
  rclcpp::Subscription<Undock::Impl::GoalStatusMessage>::SharedPtr undocking_goal_status_sub_;

  // Goal related state
  DockGoalHandle::SharedPtr dock_goal_handle_;
  UndockGoalHandle::SharedPtr undock_goal_handle_;

  // Flags to indicate if the plugins have been loaded
  bool plugins_loaded_ = false;
  bool server_failed_ = false;

  QVBoxLayout * main_layout_{nullptr};
  QHBoxLayout * info_layout_{nullptr};
  QVBoxLayout * feedback_layout_{nullptr};
  QHBoxLayout * dock_id_layout_{nullptr};
  QHBoxLayout * dock_type_layout_{nullptr};
  QHBoxLayout * dock_pose_layout_{nullptr};
  QHBoxLayout * nav_stage_layout_{nullptr};

  QComboBox * dock_type_{nullptr};
  QPushButton * docking_button_{nullptr};
  QPushButton * undocking_button_{nullptr};
  QCheckBox * use_dock_id_checkbox_{nullptr};
  QCheckBox * nav_to_staging_checkbox_{nullptr};

  QLabel * docking_goal_status_indicator_{nullptr};
  QLabel * docking_feedback_indicator_{nullptr};
  QLabel * docking_result_indicator_{nullptr};

  QLineEdit * dock_id_{nullptr};
  QLineEdit * dock_pose_x_{nullptr};
  QLineEdit * dock_pose_y_{nullptr};
  QLineEdit * dock_pose_yaw_{nullptr};

  // The current state of the docking and undocking actions
  bool docking_in_progress_ = false;
  bool undocking_in_progress_ = false;
  bool use_dock_id_ = false;

  QStateMachine state_machine_;
  InitialDockThread * initial_thread_;

  QState * pre_initial_{nullptr};
  QState * idle_{nullptr};
  QState * docking_{nullptr};
  QState * undocking_{nullptr};
  QState * canceled_docking_{nullptr};
  QState * canceled_undocking_{nullptr};
};

class InitialDockThread : public QThread
{
  Q_OBJECT

public:
  explicit InitialDockThread(
    rclcpp_action::Client<nav2_msgs::action::DockRobot>::SharedPtr & dock_client,
    rclcpp_action::Client<nav2_msgs::action::UndockRobot>::SharedPtr & undock_client)
  : dock_client_(dock_client), undock_client_(undock_client)
  {}

  void run() override
  {
    while (!dock_active_) {
      dock_active_ = dock_client_->wait_for_action_server(std::chrono::seconds(1));
    }

    while (!undock_active_) {
      undock_active_ = undock_client_->wait_for_action_server(std::chrono::seconds(1));
    }

    if (dock_active_) {
      emit dockingActive();
    } else {
      emit dockingInactive();
    }

    if (undock_active_) {
      emit undockingActive();
    } else {
      emit undockingInactive();
    }
  }

signals:
  void dockingActive();
  void dockingInactive();
  void undockingActive();
  void undockingInactive();

private:
  rclcpp_action::Client<nav2_msgs::action::DockRobot>::SharedPtr dock_client_;
  rclcpp_action::Client<nav2_msgs::action::UndockRobot>::SharedPtr undock_client_;
  bool dock_active_ = false;
  bool undock_active_ = false;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__DOCKING_PANEL_HPP_
