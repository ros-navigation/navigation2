// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef MISSION_EXECUTION__MISSIONEXECUTION_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mission_execution/MissionPlan.hpp"

class MissionExecution : public rclcpp::Node
{
public:
  MissionExecution();
  virtual ~MissionExecution();

  /**
   * @brief Execute a Mission Plan
   */
  void executeMission(const MissionPlan * missionPlan);

  /**
   * @brief Cancel the current, in-flight Mission, which must have been previously
   * starting using executeMission.
   */
  void cancelMission();

private:
  void onCmdReceived(const std_msgs::msg::String::SharedPtr msg);

private:
  const MissionPlan * missionPlan_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmdSub_;
};

#endif  // MISSION_EXECUTION__MISSIONEXECUTION_HPP_
