// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "mission_execution/MissionExecutionStateMachine.hpp"

#include <iostream>
#include "yasmine.hpp"

typedef sxe::SX_UNIQUE_PTR<sxy::async_state_machine> state_machine_uptr;

typedef struct yasmine_data {
  state_machine_uptr sm;
} yasmine_data;

static void
handleUnhandledEvent(const sxy::event& event)
{
  std::cout << "Unhandled state machine event: '" << event.get_name() << "'" << std::endl;
}

MissionExecutionStateMachine::MissionExecutionStateMachine(
  MissionExecutionStateMachineBehavior * behavior)
: behavior_(behavior)
{
  impl_ = new yasmine_data;
  sxy::version::log_version();

  initStateMachine();

  if (checkStateMachineForDefects())
    throw("MissionExecution state machine has defects");
}

MissionExecutionStateMachine::~MissionExecutionStateMachine()
{
}

void MissionExecutionStateMachine::run()
{
  //RCLCPP_INFO(this->get_logger(), "MissionExecutionStateMachine::run()\n");
  std::cout << "MissionExecutionStateMachine::run()\n";
}

void MissionExecutionStateMachine::fireEvent(const MissionExecutionEvent eventToFire)
{
  //RCLCPP_INFO(this->get_logger(), "MissionExecutionStateMachine::fireEvent: %d\n", eventToFire);
  std::cout << "MissionExecutionStateMachine::fireEvent: " << eventToFire << "\n";
}

void MissionExecutionStateMachine::halt()
{
  //RCLCPP_INFO(this->get_logger(), "MissionExecutionStateMachine::halt\n");
  std::cout << "MissionExecutionStateMachine::halt\n";
}

void MissionExecutionStateMachine::initStateMachine()
{
  std::string nm("MissionExecution");
  const std::string& name = nm;

  // Create the state machine
  state_machine_uptr state_machine = SX_MAKE_UNIQUE<sxy::async_state_machine>(name);

  // Create a single "region" to contain the state machine 
  sxy::composite_state& root_state = state_machine->get_root_state();
  sxy::region& main_region = root_state.add_region("main region");

  // Add the initial state, required by all state machines
  sxy::initial_pseudostate& initial_pseudostate = main_region.add_initial_pseudostate("initial");

  // Add the states
  sxy::simple_state& simple_state_ready = main_region.add_simple_state("Ready", 
    Y_BEHAVIOR_METHOD2(behavior_, &MissionExecutionStateMachineBehavior::doReadyState));

  sxy::simple_state& simple_state_executing = main_region.add_simple_state("Executing", 
    Y_BEHAVIOR_METHOD2(behavior_, &MissionExecutionStateMachineBehavior::doExecutingState));

  sxy::simple_state& simple_state_canceling = main_region.add_simple_state("Canceling", 
    Y_BEHAVIOR_METHOD2(behavior_, &MissionExecutionStateMachineBehavior::doCancelingState));

  sxy::simple_state& simple_state_recovering = main_region.add_simple_state("Recovering", 
    Y_BEHAVIOR_METHOD2(behavior_, &MissionExecutionStateMachineBehavior::doRecoveringState));

  sxy::simple_state& simple_state_aborting = main_region.add_simple_state("Aborting", 
    Y_BEHAVIOR_METHOD2(behavior_, &MissionExecutionStateMachineBehavior::doAbortingState));

  // Add the state transitions

  // The default state transition to Ready
  state_machine->add_transition(sxy::Y_COMPLETION_EVENT_ID, initial_pseudostate, simple_state_ready);

  // Transitions from Ready
  state_machine->add_transition(EVENT_EXECUTE_MISSION, simple_state_ready, simple_state_executing);

  // Transitions from Executing
  state_machine->add_transition(EVENT_CANCEL_MISSION, simple_state_executing, simple_state_canceling);
  state_machine->add_transition(EVENT_EXECUTE_RECOVERY, simple_state_executing, simple_state_recovering);
  state_machine->add_transition(EVENT_EXECUTION_FAILED, simple_state_executing, simple_state_aborting);
  state_machine->add_transition(EVENT_MISSION_EXECUTED, simple_state_executing, simple_state_ready);

  // Transitions from Canceling
  state_machine->add_transition(EVENT_MISSION_CANCELED, simple_state_canceling, simple_state_ready);

  // Transitions from Recovering
  state_machine->add_transition(EVENT_RECOVERY_SUCCESSFUL, simple_state_recovering, simple_state_executing);
  state_machine->add_transition(EVENT_RECOVERY_FAILED, simple_state_recovering, simple_state_aborting);

  // Transitions from Aborting
  state_machine->add_transition(EVENT_MISSION_FAILED, simple_state_aborting, simple_state_ready);
}

bool MissionExecutionStateMachine::checkStateMachineForDefects()
{
  yasmine_data *impl = (yasmine_data *) impl_;

  sxy::state_machine_defects defects;
  const bool state_machine_has_no_defects = impl->sm->check(defects);

  if (!state_machine_has_no_defects)
    sxy::write_defects_to_log(defects);

  return state_machine_has_no_defects;
}
