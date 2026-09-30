# Fix Large-Angle Circling Bug + Upgrade to 2DOF PID

## Root Cause

The circling bug is in the **trapezoidal profiler** (lines 468–503 of [pid_controller.c](file:///c:/Users/dontm/Documents/mojCodexstuff/ActiveGithub/motionsim/motionsimbot/main/pid_controller.c#L468-L503)). `s_profiled_targets[i]` is wrapped to 0–360 via `while` loops, but `shortest_angle_diff` between the **wrapped** profiled target and the **wrapped** final target can flip sign at 180°, causing the profiler to oscillate direction instead of converging. The 25° lead-clamp (line 508) prevents runaway but also prevents recovery — the motor chases its tail.

Additionally, targets of exactly 0° or 360° are problematic since the AS5600 can read values like 359.95° or 0.05° but never exactly 0/360, causing permanent dither.

## Proposed Changes

All changes are in [pid_controller.c](file:///c:/Users/dontm/Documents/mojCodexstuff/ActiveGithub/motionsim/motionsimbot/main/pid_controller.c) and [pid_controller.h](file:///c:/Users/dontm/Documents/mojCodexstuff/ActiveGithub/motionsim/motionsimbot/main/pid_controller.h).

### 1. Unwrapped-Angle Profiler (Bug Fix — **critical**)

Switch the profiler from wrapped 0–360 arithmetic to **unwrapped** continuous angles internally:

- Add `s_unwrapped_actuals[]` and `s_unwrapped_profiled[]` state arrays  
- On each tick, unwrap the raw AS5600 reading relative to the previous unwrapped value (detect ±180° boundary crossings)
- Profiler operates entirely in unwrapped space — no `while` wrap loops
- `shortest_angle_diff` is **not** used for profiler distance; use plain subtraction on unwrapped values
- Only re-wrap to 0–360 for display/telemetry (`pid_get_actual_angle`, `pid_get_profiled_target`)

This eliminates the 180° sign-flip entirely.

### 2. Safe 0°/360° Target Clamping

In `pid_set_target_angle()`:
- Clamp targets within a small margin of the singularity: if target < 0.5°, set to 0.5°; if target > 359.5°, set to 359.5°
- This avoids the AS5600 never-reaching-exactly-zero problem

### 3. 2DOF PID Math Upgrade

Replace the current P+I+D+FF+stiction sum (lines 676–706) with proper 2DOF formulation:

```c
// 2DOF: P acts on weighted setpoint error, D acts on measurement only
float p_term = s_params.kp * (beta * profiled_error_setpoint_weighted - measurement_deviation);
// Actually simplified: Kp * (beta * r - y) where r=profiled_target, y=actual
float p_term = s_params.kp * (BETA * s_profiled_targets[i] - s_unwrapped_actuals[i]); // unwrapped
// ... but keeping the existing error-based form and just adding beta weighting:

float setpoint_err = beta * error;  // weighted for proportional
float full_err = error;              // full for integral

float p_term = s_params.kp * setpoint_err;
float d_term = -(s_params.kd * s_d_filtered[i]);  // already on measurement ✓

// Back-calculation anti-windup on integral:
float raw_output = p_term + i_term_prev + d_term + ff_term + stiction;
float sat_output = clamp(raw_output, -clamp_limit, clamp_limit);
s_integrals[i] += (s_params.ki * full_err * dt) + (K_AW * (sat_output - raw_output) * dt);
s_integrals[i] = clamp(s_integrals[i], -integral_max, integral_max);
```

**New params added to `pid_params_t`:**
- `float beta;` — setpoint weight (default 0.6)
- `float k_aw;` — anti-windup back-calc gain (default 0.5)

### 4. Asymmetric Friction Compensation

Replace single `min_duty` with:
```c
float breakaway_fwd;   // static friction pulse, forward
float breakaway_rev;   // static friction pulse, reverse  
float coulomb_fwd;     // continuous friction offset, forward
float coulomb_rev;     // continuous friction offset, reverse
```

Keep backward-compatible: if `breakaway_fwd == 0`, fall back to existing `min_duty` behavior. Only apply breakaway pulse when motor is commanded to move but velocity ≈ 0, with a timer to prevent continuous application.

### 5. Deadband + Velocity Gate

Current deadband (line 669) only checks position. Add velocity gate:
```c
if (traj_dist < 0.15f && dist_to_target <= deadband_deg 
    && fabsf(s_d_filtered[i]) < 3.0f   // actual shaft velocity near zero
    && fabsf(s_profiled_vels[i]) < 0.5f) {
    motor_set_duty(motor_id, 0);
}
```

### NVS Changes

Add new fields to save/load for `beta`, `k_aw`, `breakaway_fwd/rev`, `coulomb_fwd/rev`. Existing NVS keys are untouched for backward compat.

## Verification Plan

### Build
```
cd motionsimbot && idf.py build
```

### Manual Verification
- Command 110→10 (previously circling) — should now travel shortest path and stop
- Command 160→110 — should still work as before  
- Command to 0° — should clamp to 0.5° and settle
- Command to 180° — should work fine (not a singularity for the sensor, just for wrapping)
- Large moves like 350→10 (crossing 0°) — should take 20° short path, not 340° long way

> [!IMPORTANT]
> The unwrapped-angle fix (#1) is the **critical** change that fixes circling. Items #3-#4 are quality upgrades. Want me to do just #1+#2 first for a quick fix, or the full set?
