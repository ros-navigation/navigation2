# nav2_mppi_controller — working notes

This is a **customized fork** of the upstream Nav2 MPPI controller. Before changing anything, assume
a file may already diverge from upstream and read it rather than relying on knowledge of upstream
Nav2. `git log --oneline` in `src/navigation2` shows the fork commits.

## Where things live

The control cycle starts at `MPPIController::computeVelocityCommands` (`src/controller.cpp:92`),
which calls `Optimizer::evalControl` (`src/controller.cpp:109`). Inside the optimizer the per-cycle
sequence that matters is:

1. `prepare()` copies the robot velocity into the state: `state_.speed` comes from the measured
   `robot_speed`, or from `last_command_vel_` when `open_loop` is true (`src/optimizer.cpp:303`).
2. `generateNoisedTrajectories()` calls `noise_generator_.setNoisedControls(state_, ...)` then
   `noise_generator_.generateNextNoises()`.
3. Critics score the rollouts, then `updateControlSequence()` folds in the `gamma` control-cost terms
   and does the softmax weighting.

`NoiseGenerator` owns all sampling-deviation logic. `Optimizer` owns parameters and the cost math.

## Fork feature: adaptive sampling deviations

All three sampling deviations can decay exponentially as the robot speeds up. The curve peaks at
standstill, so a slow robot samples a wider spread and a fast robot a narrow, stable one.

```
f(speed) = (base_std - decay_to) * e^(-strength * speed) + decay_to
```

Parameters, all under the controller namespace, all dynamically reconfigurable:

| axis | strength | target | driven by |
| --- | --- | --- | --- |
| angular | `advanced.wz_std_decay_strength` | `advanced.wz_std_decay_to` | linear speed magnitude, `hypot(vx, vy)` on holonomic bases |
| linear x | `advanced.vx_std_decay_strength` | `advanced.vx_std_decay_to` | measured `abs(vx)` |
| linear y | `advanced.vy_std_decay_strength` | `advanced.vy_std_decay_to` | measured `abs(vy)`, holonomic bases only |

A strength at or below zero disables that axis. Defaults are `-1.0` and `0.0`, so the whole feature
is off unless configured. The angular axis came first and keys off the speed magnitude on purpose;
the two linear axes were added later and key off their own axis. Do not "unify" that without asking,
it is a deliberate difference.

The implementation is `NoiseGenerator::computeAdaptiveStds` in `src/noise_generator.cpp`, called from
`setNoisedControls` under the noise lock. Two static helpers carry the math, `applyStdDecay` and
`shouldApplyStdDecay`. Results land in `vx_std_adaptive_`, `vy_std_adaptive_`, `wz_std_adaptive_`
and are read by the optimizer through the three `get*StdAdaptive()` accessors, which feed the `gamma`
denominators in `updateControlSequence`.

### Two behaviors that surprise people

**The adaptive deviation does not widen the sampled noise unless `regenerate_noises` is true.** That
parameter defaults to false, in which case the noise arrays are drawn once in `initialize()` and
`reset()` and never redrawn. Refreshing a `std::normal_distribution` only affects the next
`generateNoisedControls()` call, and that only runs from the noise thread, which only exists when
`regenerate_noises` is true. With the default, the adaptive value still scales the `gamma` control
cost every cycle, which changes behavior but not the sampling spread. Anyone tuning this on a robot
needs to know which of the two effects they are actually getting.

**With `regenerate_noises` true there is a one-cycle lag.** `setNoisedControls` applies noises that
were generated with the previous deviation, and only then updates the distribution for the next
cycle.

### `decay_to` must be greater than zero

The gamma control cost weight is `gamma / std^2`, so a deviation heading to zero sends that weight to
infinity, the costs to NaN and the control sequence with them. With `wz_std = 0.4` and strength 3 the
weight is 0.094 at standstill, 1.5e4 at 2 m/s and literally infinite by 20 m/s. So
`shouldApplyStdDecay()` rejects `decay_to <= 0` the same way it rejects an out-of-range one, and the
decay is skipped rather than applied with a floor that breaks the cost function.

### Backward compatibility in `updateControlSequence()`

The guards around the three control cost terms are upstream's and stay that way: none on the forward
axis, `sampling_std.wz > 0` on the angular one from PR #5110, and the holonomic check on the lateral
one. Do not add or move guards here. A fork pass that guarded on the adaptive value instead could
drop a whole cost term, which is a behavior change upstream configurations never asked for.

What the fork does add is the deviation each term divides by. `effective_std()` takes the adaptive
value when it is positive and falls back to the configured one otherwise, since a non positive
adaptive value means the dynamic calculation is disabled for that axis. With no decay configured the
adaptive value equals the configured one, so the arithmetic is identical to upstream.

A negative `vx_std` stays legal, as upstream leaves it. It is squared in the denominator so the sign
never reaches the cost, and nobody has complained. Note it does reach `std::normal_distribution`,
where a negative deviation is out of contract and libstdc++ draws samples of that magnitude with the
sign mirrored, so `vx_std = -123` means very large noise rather than no noise. Left alone on purpose.

### Tuning implication

Because the curve peaks at standstill, `vx_std` is the **boosted low-speed** value and
`vx_std_decay_to` the cruising value. Enabling the feature means raising `vx_std` above whatever was
tuned without it and setting `decay_to` to the old value. Same for the other two axes.

### Known rough edges, left deliberately

- The `validate*StdDecayConstraints()` wrappers treat a disabled decay as **valid**, while
  `shouldApplyStdDecay()` treats it as **do not apply**. Different questions, same bounds check
  underneath. Easy to misread.
- An out-of-bounds `decay_to` falls back to the static deviation. `Optimizer::getParams()` warns
  about it, since that is where a logger exists. The older warning inside `NoiseGenerator::reset()`
  is still commented out, has no logger to use, and names the max angular velocity rather than the
  sampling deviation it compares against. Delete it rather than revive it.
- The `stddev() != adaptive` guards before rebuilding each distribution are exact float comparisons,
  so with decay enabled they essentially always fire. Cheap, but they buy nothing.

## Parameters

Everything goes through `ParametersHandler::getParamGetter(name_)` in `Optimizer::getParams()`.
`getParam` declares the parameter if needed and registers it as **dynamic** by default, and any
change fires the post-callback that calls `Optimizer::reset()`, which calls `NoiseGenerator::reset()`.
So a new parameter is dynamically reconfigurable for free, and reset-safe state belongs in `reset()`.

Only `vx_max`, `vx_min`, `vy_max` and `wz_max` have a pre-callback guard, which rejects dynamic
changes while a speed limit is active (`src/optimizer.cpp:85`). The advanced decay parameters have no
range validation registered; their bounds are checked at use time instead.

`OptimizerSettings` and the structs it holds use **aggregate initializers**
(`models/optimizer_settings.hpp`). Adding a field to `ControlConstraints`, `AdvancedConstraints` or
`SamplingStd` means updating the brace list, or the build breaks in a confusing place.

## Build and test

Build the whole workspace with `build.sh` at the workspace root. It builds Debug with
`--symlink-install`. **Do not delete `build/` or `install/` directories**; report the failure and let
Vahap run the clean rebuild.

Single package, which is fine to run freely:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav2_mppi_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select nav2_mppi_controller
colcon test-result --verbose --test-result-base build/nav2_mppi_controller
```

A healthy run is 292 tests, 0 failures, 63 skipped. The skips are pre-existing.

Lint runs as part of `colcon test`: uncrustify, cpplint, lint_cmake, xmllint. **Lines must be 100
columns or fewer**, and cpplint will fail the build over a single long line. Check before building:

```bash
find src include test -name '*.cpp' -o -name '*.hpp' | xargs awk 'length > 100 {print FILENAME":"FNR}'
```

Long Doxygen comment lines in `models/constraints.hpp` predate this rule and are tolerated; match the
surrounding style there rather than reflowing the file.

## Testing conventions

`test/noise_generator_test.cpp` builds the generator directly, no ROS graph needed beyond a
`LifecycleNode` and a `ParametersHandler`. The pattern is: declare
`test_name.regenerate_noises`, construct the handler, then `initialize(settings, is_holonomic,
"test_name", &handler)` followed by `reset(settings, is_holonomic)`.

`settings` must be a **non-const lvalue**; the fork's `initialize` and `reset` take
`OptimizerSettings &`, unlike upstream. Changing those signatures breaks every test call site plus
two calls in `src/optimizer.cpp`.

`computeAdaptiveStds` and the `get*StdAdaptive` accessors are public specifically so tests can drive
them without going through a full optimizer cycle. Set `state.speed.linear.x` / `.y`, call
`computeAdaptiveStds(state)`, assert on the accessors.

Validation helpers read `settings_`, the generator's own copy, so **call them after
`initialize()`/`reset()`**, not before. Before that the copy is default-constructed and the
assertions pass vacuously.

## Build failures that are not code problems

Seen in this workspace, worth recognizing before hunting for a bug:

- **Corrupted object file.** A link failed with undefined references to `std::fufction`, a single bit
  flip of `function` in the symbol table of one `.o`. Confirm with
  `strings -a <file> | grep fufction`, then rebuild.
- **Internal compiler error, segfault.** After a glibc or toolchain upgrade via apt, stale artifacts
  built against the old headers can crash GCC. Check `/var/log/apt/history.log` against artifact
  timestamps. A clean rebuild is the fix.
- Packages reported as **Aborted** by colcon were interrupted by another package's failure. They did
  not fail on their own.

## Docs

`README.md` carries the parameter table. Every fork-added parameter has a row there, with the decay
function written out and a Wolfram Alpha link to visualize it. Keep new parameters documented the
same way. The example YAML further down the README and the bringup parameter files list only
`vx_std`, `vy_std`, `wz_std`, not the advanced parameters.

### Voice

Write doc comments plainly. Drop qualifiers and edge-case caveats the reader infers on their own, and
do not invent precision you have no evidence for. Vahap rewrote an earlier pass of these comments as
"too scientific": a formula belongs in brackets as a hint rather than spelled out with every branch,
and tuning numbers that were never tested on the robot do not belong in the header at all.

A `@brief` runs to the end of its paragraph, so leave a blank `*` line after the opening sentence.
Without it the whole comment lands in the member summary table. Cross-reference sibling members with
`#member_name`, which Doxygen renders as a link, rather than backticks, which render as code.
