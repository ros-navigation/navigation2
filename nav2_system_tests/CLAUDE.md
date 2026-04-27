# nav2_system_tests — assistant guide

End-to-end Gazebo + Nav2 integration tests. **Heavy and flaky** — `tools/run_test_suite.bash` deliberately skips this package's tests except for `test_dynamic_obstacle` (run via `tools/ctest_retry.bash` with retries). Lints still run unconditionally.

## What's here
- `src/system/` — full launch-and-navigate integration tests.
- `src/behavior_tree/`, `src/behaviors/`, `src/planning/`, `src/localization/`, `src/waypoint_follower/`, `src/dynamic_obstacle/` — feature-scoped integration tests.
- Each test is a Python launch script that boots a TB3 world, runs Nav2, and asserts via the `BasicNavigator` API.

## Running locally
Requires Gazebo / sim setup. From workspace root:

```bash
colcon build --packages-select nav2_system_tests --symlink-install
source install/setup.bash
colcon test --packages-select nav2_system_tests
# or single test:
ctest --test-dir build/nav2_system_tests -R test_<name> -V
```

Most CI runs **only** the canonical subset; do not assume all tests pass locally.

## How to add a system test
1. New test under `src/<area>/`. Mirror the pattern of an existing test (launch description + Python test driver).
2. Register in `CMakeLists.txt` via `add_launch_test`.
3. **Decide gating.** If the test is flaky (sim variance), don't add it to the canonical subset in `tools/run_test_suite.bash` — skip-listed by default.

## Lint
This package's lints **do** run in CI even though most tests are skipped — keep `.py` files mypy/flake8/pep257-clean.

## Docs to update
- Coverage report (`tools/code_coverage_report.bash`) if you add tests that significantly shift coverage.
- No user-facing docs typically; this package is internal QA.

## Pitfalls
- Sim variance is a real source of flakes — set generous timeouts and use `BasicNavigator` polling rather than fixed sleeps.
- Tests must clean up Gazebo / spawned nodes; orphaned sim processes break later tests on the same runner.
- Adding to `tools/run_test_suite.bash`'s allow-list is a CI-impacting change — coordinate with maintainers.
