# BT Navigation Failure Audit

This note captures concrete failure modes that were found in BT navigation execution paths,
how they previously propagated, and what was changed.

## 1) Goal replacement during active `NavigateThroughPoses` with TF issues

- Scenario: A preempting goal arrives while navigation is active, but one or more new poses cannot
  be transformed due to delayed/missing TF.
- Previous propagation:
  - `NavigateThroughPosesNavigator::onPreempt()` called `initializeGoalPoses()`.
  - On transform failure it threw a `std::runtime_error`.
  - The exception escaped through the BT loop and was converted into a generic BT failure.
  - Result: active navigation could fail due to a bad replacement goal instead of rejecting that
    pending goal and continuing.
- Fix:
  - Preempt transform failure now rejects only the pending goal and keeps current navigation active.
  - This is now consistent with `NavigateToPose` preempt behavior.

## 2) Terminal BT status changes missing from introspection logs

- Scenario: Tree reaches terminal `SUCCESS` or `FAILURE`.
- Previous propagation:
  - BT topic logs were flushed only during loop iterations while the tree was still running.
  - Final status transitions could be buffered and never published.
  - Result: debugging terminal failures/successes from BT event logs could be incomplete.
- Fix:
  - Added a post-run flush in `BtActionServer::executeCallback()` so final transitions are always
    published.

## 3) BT fails without a populated error code/message

- Scenario: Tree returns `FAILED`, but no node-level error code was set on blackboard.
- Previous propagation:
  - Action result could terminate with failure but empty `error_msg` and `error_code == NONE`.
  - Result: operationally confusing failure reports.
- Fix:
  - Added explicit terminal warnings in `BehaviorTreeEngine` for `FAILURE` and unexpected `IDLE`.

## 4) Navigation action BT node missing required goal input

- Scenario: `NavigateToPoseAction` or `NavigateThroughPosesAction` ticks without required goal input.
- Previous propagation:
  - Node logged an error but still sent a default-constructed action goal.
  - Result: empty/invalid goals could be sent, obscuring root cause.
- Fix:
  - Nodes now fail fast (`should_send_goal_ = false`) and set explicit `UNKNOWN` error outputs.

## 5) Recovery loops were difficult to interpret during failures

- Scenario: primary child fails and recovery child is attempted repeatedly.
- Previous propagation:
  - Retry transitions had limited runtime logging context.
- Fix:
  - Added concise logs in `RecoveryNode` for primary failure, recovery attempt, recovery success,
    recovery failure, and retry exhaustion.

## 6) Preempt-without-pending-goal warning could spam the log at tick rate

- Scenario: A preempt is requested but `get_pending_goal()` returns null on a given loop iteration,
  and the condition persists across several BT loop iterations (ticking at up to `bt_loop_duration_`
  frequency) before resolving.
- Previous propagation:
  - `BtActionServer::executeCallback()`'s `on_loop` logged this case with `RCLCPP_WARN` unconditionally
    on every iteration where the condition held, unbounded by time.
  - Result: log flooding and unnecessary overhead on the BT tick hot path.
- Fix:
  - Switched to `RCLCPP_WARN_THROTTLE` (2s), matching the throttle convention already used in
    `NavigateToPose`/`NavigateThroughPoses` `onLoop()` for TF-unavailable warnings.

## 7) `RecoveryNode::tick()` constructed a logger on every tick regardless of outcome

- Scenario: `RecoveryNode` is ticked at full BT rate during normal, non-failing navigation (the
  common case), not just on retries/failures.
- Previous propagation:
  - `tick()` called `rclcpp::get_logger("RecoveryNode")` unconditionally at the top of every tick,
    even though the actual `RCLCPP_DEBUG`/`RCLCPP_ERROR` calls were already correctly gated to
    failure/success transitions only.
  - Result: avoidable per-tick overhead independent of whether any diagnostic message fired,
    violating the requirement that new diagnostics add no steady-state tick-path cost.
- Fix:
  - Logger is now a `rclcpp::Logger logger_` member, constructed once in the header's default
    member initializer and reused across ticks instead of being rebuilt each call.

## Plugin API/ABI stability check

- No public virtual method signatures, base-class interfaces (e.g. `BtActionNode`, `BtServiceNode`),
  or constructor signatures used by pluginlib-loaded, out-of-tree BT plugins were changed. The
  `RecoveryNode::logger_` addition is a private member on a concrete leaf control node (not a base
  class third parties inherit from), so it does not affect any out-of-tree plugin's API surface.

## Follow-up intentionally postponed

- Add end-to-end system tests in `nav2_system_tests` that assert complete navigator action results
  and BT event logs under real TF delay/intermittent localization conditions.
- Add optional structured metadata in BT status log events for recovery retry counters to improve
  offline traceability.
