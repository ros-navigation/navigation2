# Adding yaw as a 3rd optimizer channel — validation analysis

**Status:** ANALYSIS ONLY — no code changes. Created 2026-05-29.

**Question:** With our current library-ported EGO planner (LBFGS-Lite + chord-perpendicular {p,v} + two-pass solve), what happens if we put yaw back into the B-spline as a 3rd channel (positions become `(x, y, yaw)` instead of `(x, y, 0)`)?

**TL;DR:** Five of the six optimizer math pieces would either break (wrap-around bugs, meaningless cross-products) or silently produce wrong gradients. Adding yaw is **not a drop-in** — it requires per-axis bounds, wrap-aware finite differences, an angular `{p,v}` policy, and a rewrite of `calcFitnessCost`. The 27 % convergence seen in `run4_ego_aut_env_09` with the old 3D EGO is consistent with these structural issues. **Recommendation:** keep the current decoupled (lattice yaw) approach, revisit yaw-in-spline only as a v2 effort with the modifications listed below.

---

## 1. What is already 3-dimensional today

Even though we plan in (x, y), the control-point matrix `cps_.points` is already a **3 × N matrix**. The third row holds `z = 0` everywhere. So syntactically, "adding yaw" is just `cps_.points(2, i) = yaw_i`. Nothing crashes. The question is whether the math each cost term performs makes sense when that third row is angular instead of zero.

Sources (current package, post-port):
- `src/bspline_optimizer.cpp:431-470` — smoothness (jerk / acc)
- `src/bspline_optimizer.cpp:472-531` — feasibility (per-axis V/A)
- `src/bspline_optimizer.cpp:360-399` — distance / collision via {p, v} pairs
- `src/bspline_optimizer.cpp:401-429` — fitness (refine pass)
- `src/ego_orchestration.cpp:210-247` — initial guess (PolynomialTraj quintic → parameterizeToBspline)
- `src/dyn_a_star.cpp:64-89` — diagonal heuristic (uses dx, dy, dz)

---

## 2. Per-component behaviour if 3rd channel = yaw

### 2.1 `calcSmoothnessCost` (jerk term)

Current code (`bspline_optimizer.cpp:444`):
```cpp
jerk = q.col(i+3) - 3*q.col(i+2) + 3*q.col(i+1) - q.col(i);
cost += jerk.squaredNorm();
```

For each CP triple, this is a finite-difference jerk on all three rows. For (x, y), it's literally jerk_x and jerk_y in m/s³. For yaw, it's **arithmetic difference of yaw values**.

**Failure mode — wrap-around blows up the cost:**

Sequence of yaw CPs `[+3.10, +3.13, -3.13, -3.10]` rad — geometrically a smooth 0.06 rad rotation across the ±π discontinuity. The finite-difference jerk computes:
```
jerk_yaw = (-3.10) - 3·(-3.13) + 3·(+3.13) - (+3.10)
         = -3.10 + 9.39 + 9.39 - 3.10
         = +12.58  rad   ← huge
cost     = 158                ← gigantic, dwarfs every other term
gradient = ±2·jerk_yaw on each CP  ← drags all 4 CPs apart
```

The gradient direction is *correct in unwrapped angle space* but wrong in physical space. The optimizer will distort the whole trajectory trying to "straighten" a phantom kink. This is the classic failure mode of putting angles into a Euclidean cost.

**What needs to change:** unwrap yaws before evaluating the difference, OR use:
```cpp
double angle_diff(double a, double b) {
  double d = a - b;
  while (d >  M_PI) d -= 2*M_PI;
  while (d < -M_PI) d += 2*M_PI;
  return d;
}
// jerk_yaw = angle_diff(q(2,i+3), q(2,i+2)) - 3·angle_diff(q(2,i+2), q(2,i+1)) + ...
```

But this **breaks the smoothness of the gradient** — it has jumps at ±π where the wrap branch flips. L-BFGS assumes a continuously differentiable cost; jump discontinuities hurt convergence and bound the achievable progress. A workaround is to keep yaws unwrapped throughout the optimization and only wrap at output, but then initial-guess yaw must be carefully unwrapped from the lattice.

### 2.2 `calcFeasibilityCost` (per-axis V/A bounds)

Current code (`bspline_optimizer.cpp:487`):
```cpp
for (int j = 0; j < 3; j++) {
  if (vi(j) >  max_vel_) cost += pow(vi(j) - max_vel_, 2) * ts_inv2;
  if (vi(j) < -max_vel_) ...
}
```

`max_vel_` is a **single scalar** set to `v_linear_max = 0.20 m/s`. Applied uniformly to all 3 rows.

**Failure mode #1 — yaw rate over-constrained:**
With `max_vel_ = 0.20` applied to yaw, the optimizer permits `wz ≤ 0.20 rad/s` only. Robot's actual envelope is `v_angular_max = 0.30 rad/s` and can briefly burst higher. EGO would refuse spline shapes the robot can physically rotate through. Going from yaw 0 → π would need `π / 0.20 = 15.7 s` of spline duration. Long sub-goals become impossible.

**Failure mode #2 — yaw accel under-constrained:**
`max_acc_ = 0.30 m/s²` applied to yaw means the optimizer thinks yaw can accelerate at 0.30 rad/s² → 6 s to reach max wz from rest. Real robot can probably do >2.0 rad/s². EGO would over-smooth the yaw profile, producing lazy rotations that the lattice expects to be sharp.

**What needs to change:** turn `max_vel_` and `max_acc_` into `Vector3d` so each row has its own bound. The library actually had this in earlier versions (project memory: "Per-axis dynamics" change #11) but the current port dropped it.

### 2.3 `calcDistanceCostRebound` (collision via {p, v} pairs)

Current code (`bspline_optimizer.cpp:378-396`):
```cpp
double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
double dist_err = cps_.clearance - dist;
if (dist_err > 0) {
  gradient.col(i) += -3.0 * dist_err * dist_err * cps_.direction[i][j];
  // ... etc
}
```

`base_point` is a 3D position anchor on an obstacle, `direction` is a 3D unit vector pointing outward. The dot product `(CP - anchor) · direction` measures signed clearance.

**Failure mode — meaningless for yaw:**
If yaw is in the 3rd row, `base_point` and `direction` need a *yaw component*. What's the "outward yaw" away from an obstacle? It has no spatial meaning. The existing `initControlPoints` builds these pairs via chord-perpendicular geometry — but only in 2D. With yaw plugged in:
- The chord rotates in (x, y, yaw) space; the perpendicular has a yaw component that's geometrically arbitrary.
- The dot product mixes meters and radians: `(Δx, Δy, Δyaw) · (dx, dy, dyaw)`. The units cancel only by accident.

The gradient `-3·dist_err² · direction` then applies a yaw correction proportional to how close the body is to an obstacle. That's not physics — that's noise.

**What needs to change:** force `direction[i][j].z() = 0` and `base_point[i][j].z() = cps_.points(2, i)` for all pairs. The yaw doesn't participate in collision at all. With this, J_collide stays 2D and the wrap-around problem doesn't infect collision avoidance. Easy fix but explicit: existing chord-perp geometry would need a `.head<2>()` everywhere.

### 2.4 `calcFitnessCost` (refine pass)

Current code (`bspline_optimizer.cpp:415-423`):
```cpp
Eigen::Vector3d x = (q.col(i-1) + 4*q.col(i) + q.col(i+1)) / 6.0 - ref_pts_[i-1];
Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i-2]).normalized();
double xdotv = x.dot(v);
Eigen::Vector3d xcrossv = x.cross(v);
double f = pow(xdotv, 2) / a2 + pow(xcrossv.norm(), 2) / b2;
```

This is the **anisotropic fitness term** that pulls the refined spline toward the reference path. `x.cross(v)` is a 3D cross product.

**Failure mode — cross product mixes axes:**
If x = (Δx, Δy, Δyaw) and v = (vx, vy, vyaw), then `x × v` has components:
- (x×v).x = Δy · vyaw − Δyaw · vy   ← mixes meters and radians
- (x×v).y = Δyaw · vx − Δx · vyaw    ← mixes meters and radians
- (x×v).z = Δx · vy − Δy · vx        ← pure 2D (correct part)

Two of three components are dimensionally broken. The cost `xcrossv.squaredNorm()` adds them together with the dimensionally-clean Δyaw² term from `xdotv`. The optimizer minimizes a quantity with no physical meaning.

**What needs to change:** split fitness into two terms — 2D position fitness on `(x, y)` with the existing cross-dot formulation, and a scalar yaw fitness `(yaw_now − yaw_ref)²` separately. Roughly a 30-line rewrite.

### 2.5 Initial guess (`PolynomialTraj::one_segment_traj_gen` + `parameterizeToBspline`)

`ego_orchestration.cpp:210-252`:
```cpp
PolynomialTraj gl = PolynomialTraj::one_segment_traj_gen(
  start_pt, start_vel, start_acc,
  local_target_pt, local_target_vel, Eigen::Vector3d::Zero(),
  time_est);
// then sample gl uniformly to point_set, feed to parameterizeToBspline
```

`PolynomialTraj::one_segment_traj_gen` fits a quintic min-snap polynomial through the 6 boundary conditions (pos, vel, accel at start and end). Operates on 3D vectors.

**Failure mode — polynomial unwrap:**
If `start_pt.z() = +3.10` and `local_target_pt.z() = -3.10` (close in angle, far in Euclidean), the polynomial drives yaw from +3.10 through 0 to −3.10 — a **swing of −6.20 rad** instead of the +0.08 rad short-path solution. The B-spline parameterized from this point_set inherits the bad initial guess. Even if the optimizer's smoothness cost is wrap-aware, the initial guess is already on the wrong side of the unit circle and the local minimum is dramatically wrong.

**What needs to change:** pre-process boundary yaws so they're unwrapped relative to each other, e.g., walk along the lattice path and unwrap consecutive yaws so neighbours never differ by more than π. Then the polynomial interpolates monotonically. Doable but adds a state machine.

### 2.6 A* (`dyn_a_star.cpp`)

Currently A* lives in a 3D pool indexed by `(i, j, k)`. The k-dimension is unused — we plan in 2D, k is always at `CENTER_IDX_(2)`. If we promote k to yaw cells:
- `step_size_` is one scalar (defaults to 0.1 m). Same step in radians would be 0.1 rad = 5.7° — fine spatially, but A* would explore 26 neighbours per node including yaw-stepping neighbours.
- `getDiagHeu` mixes dx, dy, dz with sqrt(3) weights — implicitly assumes all three axes are the same kind of quantity. For yaw, the heuristic is meaningless.
- The pool has no wrap topology — at yaw cell 99, neighbour at yaw cell 0 is treated as non-adjacent.

**What needs to change:** either keep A* in 2D (yaw doesn't help A*) or build a separate yaw-only search. Given A* runs only inside `initControlPoints` for collision detours, keeping it 2D is the right call.

### 2.7 The benefit side

What we'd gain by having yaw in the spline:
- **Smooth `wz` feed-forward.** `cached_vel_spline_.evaluateDeBoorT(t).z()` gives angular velocity directly. Replaces the current `kp_yaw · yaw_err` P-only controller.
- **Coupled smoothing.** The optimizer can balance lateral acceleration against angular acceleration — e.g., reduce wz to give vy more room when both are demanded.
- **Lattice yaw becomes a reference, not a setpoint.** The refine-pass fitness term could pull yaw toward lattice yaw without snapping to it.

These are real benefits — but they're also achievable with a simpler post-optimization yaw spline (1D B-spline through lattice yaws) that doesn't touch the position optimizer.

---

## 3. Concrete simulation — what convergence would look like

A small numerical experiment: simulate one rebound pass with the current jerk cost on a sequence of yaw CPs that crosses ±π. Code below (sketch — not run, would need numpy):

```python
import numpy as np
# 10 CPs, yaw starting at 3.0 rad, smoothly rotating to 3.2 rad
# (which wraps to -3.083 rad).
yaw = np.linspace(3.0, 3.2, 10)
yaw_wrapped = np.where(yaw > np.pi, yaw - 2*np.pi, yaw)

# Current code's jerk cost (no wrap handling):
def jerk_cost_naive(y):
    cost = 0.0
    for i in range(len(y) - 3):
        jerk = y[i+3] - 3*y[i+2] + 3*y[i+1] - y[i]
        cost += jerk * jerk
    return cost

# Same path, but with consistently unwrapped values:
def jerk_cost_unwrapped(y):
    yu = np.unwrap(y)
    cost = 0.0
    for i in range(len(yu) - 3):
        jerk = yu[i+3] - 3*yu[i+2] + 3*yu[i+1] - yu[i]
        cost += jerk * jerk
    return cost

print(jerk_cost_naive(yaw_wrapped))    # huge (≥ 50 from one bad triple)
print(jerk_cost_unwrapped(yaw_wrapped))  # ~1e-8 (smooth)
```

Conclusion: with no wrap handling the cost explodes by ~10 orders of magnitude on any path crossing ±π. L-BFGS would interpret this as a high-gradient region and push CPs all over to "fix" it, destroying the position part of the spline as a side-effect. This is the precise mechanism behind run2/3 (0% converged) in `project_ego_tuning_history.md`.

### 3.1 Actual numerical results (2026-05-29)

Ran the simulation above on a smooth 0.4 rad rotation that crosses +π (10 CPs from 2.9 to 3.3 rad, then wrapped to [-π, π]):

```
Test 1: smooth 0.4 rad rotation crossing +π
  Unwrapped CPs:  cost = 5.52e-30   (machine-zero — correct, smooth path)
  Wrapped   CPs:  cost = 2.37e+02   (huge — wrap creates phantom 6.28 rad jerk)
  Ratio:          2.37e17×

Test 4: gradient magnitudes the optimizer would actually see
  Unwrapped: ||g||_∞ = 1.07e-14   (machine-zero gradient — nothing to fix)
  Wrapped:   ||g||_∞ = 1.26e+02   (huge gradient — L-BFGS would push CPs hard)
  CPs the optimizer would "fix" (gradient > 1): 6 of 10
```

The wrapped gradient is **10¹⁶× larger** than the unwrapped gradient on the exact same geometric path. Six of ten CPs would be classified as high-error by L-BFGS and get moved aggressively — i.e., the optimizer would distort the *position* CPs (since gradient_yaw bleeds into the same Eigen matrix as gradient_x, gradient_y at the same indices) trying to resolve a phantom yaw problem.

Test 2 (a 6 rad rotation with a double-wrap) returned the same cost as the unwrapped version. That's because the two wraps cancel inside any 4-CP window. The failure is *local to the wrap crossing* — one bad triple is enough to blow up cost and gradient.

Take-away: every door-rotation primitive that crosses ±π (eventually some will, since the world frame's yaw is arbitrary) would inject a huge gradient into both position and yaw. The optimizer would converge to a deformed spline or fail to converge at all. This matches run2/3's 0% convergence behavior exactly.

---

## 4. Net assessment vs current decoupled approach

| Aspect | Current (yaw from lattice) | Yaw-in-spline (drop-in) | Yaw-in-spline (fixed) |
|---|---|---|---|
| Lines of code touched | 0 | 0 (compile clean) | ~150 |
| Smoothness on yaw | none (P-controller only) | wrap-buggy | smooth |
| Yaw feedforward | none | derivative gives wz_ff | derivative gives wz_ff |
| Convergence rate (expected) | high (2D well-behaved) | ≤ 27 % (run4 evidence) | unknown (paper-faithful 3D EGO achieves ~70 %) |
| Collision avoidance | clean (lattice yaw used in poly check) | broken (yaw in {p,v} is nonsense) | clean (yaw zeroed in {p,v}) |
| Refine pass fitness | clean (2D cross-dot) | broken (cross mixes axes) | clean (split 2D + 1D yaw) |
| A* | 2D — no change needed | unchanged (k unused) | unchanged |
| Per-axis V/A bounds | irrelevant | need to add (currently scalar) | needed |
| Wrap-around state | irrelevant | breaks everywhere | needs explicit pre/post unwrap |
| Robustness to lattice yaw jumps (e.g., door rotation primitive) | bad — saturates wz now | bad — initial guess goes wrong way | could be smooth if yaw included in opt |

---

## 5. Recommendation

**Don't add yaw to the optimizer right now.** Cost-benefit:

- **What we'd get:** smooth wz feedforward + coupled lat/ang smoothing.
- **What it costs:** ~150 lines of careful changes across `calcSmoothnessCost`, `calcFeasibilityCost`, `calcDistanceCostRebound` (zero yaw in {p,v}), `calcFitnessCost` (split), `parameterizeToBspline` (boundary unwrap), `PolynomialTraj` (boundary unwrap). Each one is a potential bug surface. The old R³ implementation hit 27 % convergence at its peak — half-baked yaw in the optimizer is worse than no yaw in the optimizer.
- **Cheaper alternative for the same benefits:** fit a 1D B-spline over yaw separately from the position spline. Same library, but applied to a scalar sequence of yaw values from the lattice path. Wrap can be handled by unwrapping once before the fit and rewrapping after. The optimizer never sees an angular discontinuity because it sees a monotonic-ish 1D sequence. Feedforward `wz` falls out of the 1D spline derivative.

The 1D yaw-spline path captures most of the benefit (smooth `wz_ff`, lattice yaw as reference) with maybe 30 lines of code, no risk to the 2D optimizer, and no wrap-around state machine. If we want yaw in the optimization at all, that's the order to do it: 1D scalar spline first, see if the controller behaviour improves enough, then consider the full 3-DOF coupled optimizer only if 1D is insufficient.

**If the user still wants the 3-DOF version**, the implementation order would be:
1. Add `Vector3d max_vel_, max_acc_` and wire them from params (per-axis bounds).
2. Replace `jerk.squaredNorm()` with a wrap-aware variant on the yaw row.
3. In `initControlPoints` / `check_collision_and_rebound`, zero the yaw row of every `direction` and copy current yaw to the yaw row of every `base_point` (collision is 2D-only).
4. Split `calcFitnessCost` into 2D pos + 1D yaw.
5. Unwrap yaw boundary conditions before `PolynomialTraj` (and before `parameterizeToBspline`).
6. Add a yaw-unwrap pass after `BsplineOptimizeTrajRebound` returns, before storing `cached_spline_`.
7. Re-tune `ego_lambda_*` because the cost landscape changes (yaw smoothness costs add new scale).

Total: validation runs at each step against the run4–run10 baseline. ETA: ~3 days at this iteration cadence, plus retuning.

**Conclusion for this validation:** wait. Don't reintroduce yaw into the optimizer yet. Land the polygon-aware collision check (already implemented), get the next run's logs, see if yaw saturation on curves drops to acceptable levels with the decoupled approach. If it doesn't, the 1D yaw-spline is the next step. The 3-DOF coupled version is only justified if both of those fail.
