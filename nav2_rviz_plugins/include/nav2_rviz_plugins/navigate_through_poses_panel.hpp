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

#ifndef NAV2_RVIZ_PLUGINS__NAVIGATE_THROUGH_POSES_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__NAVIGATE_THROUGH_POSES_PANEL_HPP_

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
#include <QTabWidget>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_rviz_plugins
{

struct PoseTab
{
  QWidget * widget;
  QLineEdit * frame_id_edit;
  QDoubleSpinBox * pos_x_spin;
  QDoubleSpinBox * pos_y_spin;
  QDoubleSpinBox * pos_z_spin;
  QDoubleSpinBox * yaw_spin;
};

class NavigateThroughPosesPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NavigateThroughPosesPanel(QWidget * parent = 0);
  virtual ~NavigateThroughPosesPanel();

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onSendGoal();
  void onCancel();
  void onActionNameChanged();
  void onAddPose();
  void onRemovePose();

private:
  void setupUI();
  nav_msgs::msg::Goals getCurrentPoses();
  void onActionResult(
    const rclcpp_action::ClientGoalHandle
    <nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result);
  void createPoseTab(int index);
  geometry_msgs::msg::PoseStamped getPoseFromTab(const PoseTab & tab);

  std::string current_action_name_;
  bool goal_active_;

  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle_;

  QLineEdit * action_name_edit_;
  QTabWidget * poses_tab_widget_;
  QLineEdit * behavior_tree_edit_;
  QPushButton * send_goal_button_;
  QPushButton * cancel_button_;
  QPushButton * add_pose_button_;
  QPushButton * remove_pose_button_;

  std::vector<PoseTab> pose_tabs_;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__NAVIGATE_THROUGH_POSES_PANEL_HPP_
