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

#ifndef NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_rviz_plugins/ros_action_qevent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/buffer.h"

class QPushButton;

namespace nav2_rviz_plugins
{

class InitialThread;

/// Panel to interface to the nav2 stack
class Nav2Panel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Nav2Panel(QWidget * parent = 0);
  virtual ~Nav2Panel();

  void onInitialize() override;

  /// Load and save configuration data
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void startThread();
  void onStartup();
  void onShutdown();
  void onCancel();
  void onPause();
  void onResume();
  void onResumedWp();
  void onAccumulatedWp();
  void onAccumulatedNTP();
  void onAccumulating();
  void onNewGoal(double x, double y, double theta, QString frame);
  void handleGoalSaver();
  void handleGoalLoader();
  void loophandler();
  void initialStateHandler();

private:
  void loadLogFiles();
  void onCancelButtonPressed();
  void timerEvent(QTimerEvent * event) override;
  bool isLoopValueValid(std::string & loop);

  int unique_id {0};
  int goal_index_ = 0;
  int loop_count_ = 0;
  bool store_initial_pose_ = false;
  bool initial_pose_stored_ = false;
  bool loop_counter_stop_ = true;
  std::string loop_no_ = "0";
  std::string base_frame_;

  // Call to send NavigateToPose action request for goal poses
  geometry_msgs::msg::PoseStamped convert_to_msg(
    std::vector<double> pose,
    std::vector<double> orientation);
  void startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses);
  void startNavigation(geometry_msgs::msg::PoseStamped);
  void startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses);
  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using WaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  using NavThroughPosesGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;

  // The (non-spinning) client node used to invoke the action client
  rclcpp::Node::SharedPtr client_node_;

  // Timeout value when waiting for action servers to respnd
  std::chrono::milliseconds server_timeout_;

  // A timer used to check on the completion status of the action
  QBasicTimer timer_;

  // The NavigateToPose action client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
    nav_through_poses_action_client_;

  // Navigation action feedback subscribers
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
    navigation_feedback_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr
    nav_through_poses_feedback_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
    navigation_goal_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::SharedPtr
    nav_through_poses_goal_status_sub_;

  // Tf's for initial pose
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Goal-related state
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;

  // The client used to control the nav2 stack
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_nav_;
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_loc_;

  QCheckBox * store_initial_pose_checkbox_{nullptr};

  QPushButton * start_reset_button_{nullptr};
  QPushButton * pause_resume_button_{nullptr};
  QPushButton * navigation_mode_button_{nullptr};
  QPushButton * save_waypoints_button_{nullptr};
  QPushButton * load_waypoints_button_{nullptr};
  QPushButton * pause_waypoint_button_{nullptr};

  QLabel * navigation_status_indicator_{nullptr};
  QLabel * localization_status_indicator_{nullptr};
  QLabel * navigation_goal_status_indicator_{nullptr};
  QLabel * navigation_feedback_indicator_{nullptr};
  QLabel * waypoint_status_indicator_{nullptr};
  QLabel * number_of_loops_{nullptr};

  QLineEdit * nr_of_loops_{nullptr};

  QStateMachine state_machine_;
  InitialThread * initial_thread_;

  QState * pre_initial_{nullptr};
  QState * initial_{nullptr};
  QState * idle_{nullptr};
  QState * reset_{nullptr};
  QState * paused_{nullptr};
  QState * resumed_{nullptr};
  QState * paused_wp_{nullptr};
  QState * resumed_wp_{nullptr};

  QLabel * imgDisplayLabel_{nullptr};

  // The following states are added to allow for the state of the button to only expose reset
  // while the NavigateToPoses action is not active. While running, the user will be allowed to
  // cancel the action. The ROSActionTransition allows for the state of the action to be detected
  // and the button state to change automatically.
  QState * running_{nullptr};
  QState * canceled_{nullptr};
  // The following states are added to allow to collect several poses to perform a waypoint-mode
  // navigation or navigate through poses mode.
  QState * accumulating_{nullptr};
  QState * accumulated_wp_{nullptr};
  QState * accumulated_nav_through_poses_{nullptr};

  std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
  std::vector<geometry_msgs::msg::PoseStamped> store_poses_;

  // Publish the visual markers with the waypoints
  void updateWpNavigationMarkers();

  // Create unique id numbers for markers
  int getUniqueId();

  void resetUniqueId();

  // create label string from feedback msg
  static inline QString getNavToPoseFeedbackLabel(
    nav2_msgs::action::NavigateToPose::Feedback msg =
    nav2_msgs::action::NavigateToPose::Feedback());
  static inline QString getNavThroughPosesFeedbackLabel(
    nav2_msgs::action::NavigateThroughPoses::Feedback =
    nav2_msgs::action::NavigateThroughPoses::Feedback());
  template<typename T>
  static inline std::string toLabel(T & msg);

  // round off double to the specified precision and convert to string
  static inline std::string toString(double val, int precision = 0);

  // Waypoint navigation visual markers publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
};

class InitialThread : public QThread
{
  Q_OBJECT

public:
  using SystemStatus = nav2_lifecycle_manager::SystemStatus;

  explicit InitialThread(
    std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> & client_nav,
    std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> & client_loc)
  : client_nav_(client_nav), client_loc_(client_loc)
  {}

  void run() override
  {
    SystemStatus status_nav = SystemStatus::TIMEOUT;
    SystemStatus status_loc = SystemStatus::TIMEOUT;

    while (status_nav == SystemStatus::TIMEOUT) {
      if (status_nav == SystemStatus::TIMEOUT) {
        status_nav = client_nav_->is_active(std::chrono::seconds(1));
      }
    }

    // try to communicate twice, might not actually be up if in SLAM mode
    bool tried_loc_bringup_once = false;
    while (status_loc == SystemStatus::TIMEOUT) {
      status_loc = client_loc_->is_active(std::chrono::seconds(1));
      if (tried_loc_bringup_once) {
        break;
      }
      tried_loc_bringup_once = true;
    }

    if (status_nav == SystemStatus::ACTIVE) {
      emit navigationActive();
    } else {
      emit navigationInactive();
    }

    if (status_loc == SystemStatus::ACTIVE) {
      emit localizationActive();
    } else {
      emit localizationInactive();
    }
  }

signals:
  void navigationActive();
  void navigationInactive();
  void localizationActive();
  void localizationInactive();

private:
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_nav_;
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_loc_;
};

}  // namespace nav2_rviz_plugins

#endif  //  NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
