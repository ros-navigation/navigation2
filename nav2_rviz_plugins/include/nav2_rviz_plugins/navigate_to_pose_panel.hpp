// Copyright (c) Pau Reverté 2025
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

#ifndef NAV2_RVIZ_PLUGINS__NAVIGATE_TO_POSE_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__NAVIGATE_TO_POSE_PANEL_HPP_

#include <QtWidgets>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QComboBox>
#undef NO_ERROR

#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_rviz_plugins
{

class NavigateToPosePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NavigateToPosePanel(QWidget * parent = 0);
  virtual ~NavigateToPosePanel();

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onSendGoal();
  void onCancel();
  void onActionNameChanged();

private:
  void setupUI();
  geometry_msgs::msg::PoseStamped getCurrentPose();
  void onActionResult(
    const rclcpp_action::ClientGoalHandle
    <nav2_msgs::action::NavigateToPose>::WrappedResult & result);

  std::string current_action_name_;
  bool goal_active_;

  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

  QLineEdit * action_name_edit_;
  QLineEdit * frame_id_edit_;
  QDoubleSpinBox * pos_x_spin_;
  QDoubleSpinBox * pos_y_spin_;
  QDoubleSpinBox * pos_z_spin_;
  QDoubleSpinBox * yaw_spin_;
  QLineEdit * behavior_tree_edit_;
  QPushButton * send_goal_button_;
  QPushButton * cancel_button_;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__NAVIGATE_TO_POSE_PANEL_HPP_
