// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/TaskExecutionStateMachine.hpp"

#include <iostream>
#include <string>
#include "yasmine.hpp"

typedef sxe::SX_UNIQUE_PTR<sxy::async_state_machine> state_machine_uptr;

typedef struct yasmine_data
{
  state_machine_uptr sm;
} yasmine_data;

static void
handleUnhandledEvent(const sxy::event & event)
{
  std::cout << "Unhandled state machine event: '" << event.get_name() << "'" << std::endl;
}

TaskExecutionStateMachine::TaskExecutionStateMachine(
  TaskExecutionStateMachineBehavior * behavior)
: behavior_(behavior)
{
  impl_ = new yasmine_data;

#if 0
  // Initialize the yasmine log manager
  hermes::log_manager_template<hermes::std_timestamp_policy> & log_manager =
    hermes::log_manager::get_instance();
  log_manager.set_log_level(hermes::log_level::LL_DEBUG);
  log_manager.add_logger(SX_MAKE_UNIQUE<hermes::cout_logger>());
  log_manager.run();
  sxy::version::log_version();
#endif

  initStateMachine();

  if (!isValidStateMachine()) {
    throw("TaskExecution state machine has defects");
  }
}

TaskExecutionStateMachine::~TaskExecutionStateMachine()
{
  // log_manager.halt();
}

void TaskExecutionStateMachine::run()
{
  // RCLCPP_INFO(this->get_logger(), "TaskExecutionStateMachine::run()\n");

  yasmine_data * impl = reinterpret_cast<yasmine_data *>(impl_);
  impl->sm->run();
}

void TaskExecutionStateMachine::fireEvent(const TaskExecutionEvent eventToFire)
{
  // RCLCPP_INFO(this->get_logger(), "TaskExecutionStateMachine::fireEvent: %d\n", eventToFire);

  yasmine_data * impl = reinterpret_cast<yasmine_data *>(impl_);
  impl->sm->fire_event(sxy::event_impl::create(eventToFire));
}

void TaskExecutionStateMachine::halt()
{
  // RCLCPP_INFO(this->get_logger(), "TaskExecutionStateMachine::halt\n");

  yasmine_data * impl = reinterpret_cast<yasmine_data *>(impl_);
  impl->sm->halt_and_join();
}

void TaskExecutionStateMachine::initStateMachine()
{
  std::string nm("TaskExecution");
  const std::string & name = nm;

  // Create the state machine
  state_machine_uptr state_machine = SX_MAKE_UNIQUE<sxy::async_state_machine>(name);

  // Create a single "region" to contain the state machine
  sxy::composite_state & root_state = state_machine->get_root_state();
  sxy::region & main_region = root_state.add_region("main region");

  // Add the initial state, required by all state machines
  sxy::initial_pseudostate & initial_pseudostate = main_region.add_initial_pseudostate("initial");

  // Add the states
  sxy::simple_state & simple_state_ready = main_region.add_simple_state("Ready",
      Y_BEHAVIOR_METHOD2(behavior_, &TaskExecutionStateMachineBehavior::doReadyState));

  sxy::simple_state & simple_state_executing = main_region.add_simple_state("Executing",
      Y_BEHAVIOR_METHOD2(behavior_, &TaskExecutionStateMachineBehavior::doExecutingState));

  sxy::simple_state & simple_state_canceling = main_region.add_simple_state("Canceling",
      Y_BEHAVIOR_METHOD2(behavior_, &TaskExecutionStateMachineBehavior::doCancelingState));

  sxy::simple_state & simple_state_recovering = main_region.add_simple_state("Recovering",
      Y_BEHAVIOR_METHOD2(behavior_, &TaskExecutionStateMachineBehavior::doRecoveringState));

  sxy::simple_state & simple_state_aborting = main_region.add_simple_state("Aborting",
      Y_BEHAVIOR_METHOD2(behavior_, &TaskExecutionStateMachineBehavior::doAbortingState));

  // Add the state transitions

  // The default state transition to Ready
  state_machine->add_transition(sxy::Y_COMPLETION_EVENT_ID, initial_pseudostate,
    simple_state_ready);

  // Transitions from Ready
  state_machine->add_transition(EVENT_EXECUTE_TASK, simple_state_ready, simple_state_executing);

  // Transitions from Executing
  state_machine->add_transition(EVENT_CANCEL_TASK, simple_state_executing,
    simple_state_canceling);
  state_machine->add_transition(EVENT_EXECUTE_TASK_RECOVERY, simple_state_executing,
    simple_state_recovering);
  state_machine->add_transition(EVENT_TASK_EXECUTION_FAILED, simple_state_executing,
    simple_state_aborting);
  state_machine->add_transition(EVENT_TASK_EXECUTED, simple_state_executing,
    simple_state_ready);

  // Transitions from Canceling
  state_machine->add_transition(EVENT_TASK_CANCELED, simple_state_canceling, simple_state_ready);

  // Transitions from Recovering
  state_machine->add_transition(EVENT_TASK_RECOVERY_SUCCESSFUL, simple_state_recovering,
    simple_state_executing);
  state_machine->add_transition(EVENT_TASK_RECOVERY_FAILED, simple_state_recovering,
    simple_state_aborting);

  // Transitions from Aborting
  state_machine->add_transition(EVENT_TASK_FAILED, simple_state_aborting,
    simple_state_ready);

  // Set the handler for any invalid state transitions
#ifndef SX_CPP03_BOOST
  state_machine->set_behavior_of_unhandled_event_handler(
    Y_BEHAVIOR_FUNCTION2(handleUnhandledEvent));
#else
  state_machine->set_behavior_of_unhandled_event_handler(
    sxy::behavior_function(sxe::bind(&handleUnhandledEvent, sxe::_1)));
#endif

  // Save the resulting state machine to our member variable
  yasmine_data * impl = reinterpret_cast<yasmine_data *>(impl_);
  impl->sm = sxe::move(state_machine);
}

bool TaskExecutionStateMachine::isValidStateMachine()
{
  yasmine_data * impl = reinterpret_cast<yasmine_data *>(impl_);

  sxy::state_machine_defects defects;
  const bool isValid = impl->sm->check(defects);

  if (!isValid) {
    sxy::write_defects_to_log(defects);
  }

  return isValid;
}
