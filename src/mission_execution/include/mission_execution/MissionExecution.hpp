// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef MISSION_EXECUTION__MISSIONEXECUTION_HPP_
#define MISSION_EXECUTION__MISSIONEXECUTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "mission_execution/MissionPlan.hpp"
#include "mission_execution/MissionExecutionStateMachineBehavior.hpp"
#include "mission_execution/MissionExecutionStateMachine.hpp"

class MissionExecution : public rclcpp::Node, public MissionExecutionStateMachineBehavior
{
public:
  MissionExecution();
  virtual ~MissionExecution();

  /**
   * @brief Start Mission Execution. Once the instance is initialized, the state
   * machine automatically transitions to the <b>Ready</b> state and is ready to
   * execute missions, which can be initiated using executionMission.
   */
  void start();

  /**
   * @brief Stop Mission Execution. Any in-flight missions must be canceled first,
   * using cancelMission.
   */
  void stop();

  /**
   * @brief Execute a Mission Plan
   */
  void executeMission(const MissionPlan * missionPlan);

  /**
   * @brief Cancel the current, in-flight Mission, which must have been previously
   * starting using executeMission.
   */
  void cancelMission();

  /** @name State Machine Behavior
   * Implementation of the MissionExecutionStateMachineBehavior interface
   */
  ///@{
  void doReadyState();
  void doExecutingState();
  void doCancelingState();
  void doRecoveringState();
  void doAbortingState();
  ///@}

private:
  MissionExecutionStateMachine m_MissionExecutionStateMachine;
  const MissionPlan * m_missionPlan;
};

#endif  // MISSION_EXECUTION__MISSIONEXECUTION_HPP_
