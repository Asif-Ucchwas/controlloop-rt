# ControlLoop-RT Benchmark Test Protocol

Documented before any benchmark code is written, per project convention
(same discipline as CAN-Net's Stage 5 protocol) - the plan is fixed
first, so results can't be quietly cherry-picked after the fact.

## Controllers Under Test

    1. PID/PD baseline    - Stage 1, Kp=144, Kd=6.79, Ki=0
    2. PD + Feedforward    - Stage 1, same PD gains + model-inversion feedforward
    3. MPC                 - Stage 2, n_horizon=20, velocity constraint +/-3.0 rad/s

All three reuse EXACT parameters already validated in Stages 1-2 - no
re-tuning for this benchmark, so results reflect the controllers as
actually built and documented, not a re-optimized version.

## Reference Trajectory (fixed across all trials)

Sinusoidal reference: amplitude=1.0 rad, frequency=0.5 Hz, duration=5.0s.
Same trajectory validated in Stage 1 Task 3 (feedforward comparison) and
Stage 2 Task 8 (stress test). Chosen because it is a MOVING target -
a step input would give feedforward zero opportunity to help (Stage 1
Task 3 finding: feedforward provides no benefit on step inputs since the
plant already has a built-in integrator), making a step-only comparison
unfair to two of the three controllers under test.

## Varying Conditions (2x2 grid)

    Disturbance:  {NONE, PRESENT}
      PRESENT = -80 rad/s^2 torque-equivalent term, active t=[1.0, 1.3]s
      (identical to Stage 2 Task 8's disturbance)

    Sensor noise: {NONE, PRESENT}
      PRESENT = theta noise std=0.01 rad, theta_dot noise std=0.05 rad/s
      (identical to Stage 2 Task 8's noise model)

    4 conditions: (no-dist, no-noise), (no-dist, noise), (dist, no-noise), (dist, noise)

## Trial Count

50 trials per (controller x condition) pair, matching the thesis's
50-trial rigor referenced throughout this portfolio's syllabi.

    3 controllers x 4 conditions x 50 trials = 600 total simulation runs

**What varies between trials within a condition:** only the sensor-noise
random seed (when noise is PRESENT for that condition). Disturbance
timing/magnitude stays fixed within a condition. This isolates noise's
statistical (Monte Carlo) variability cleanly, rather than conflating
two independent randomness sources into one number.

**What stays fixed across ALL 600 runs:** plant parameters (J, b, K, R
from Stage 1), controller gains/configuration, reference trajectory,
simulation timestep and duration, disturbance magnitude/timing (when
present).

## Metrics Logged Per Trial

Per-trial (raw), per Section 9 of docs/notes/controls_math_reference.md:

    - RMS tracking error            (Section 9)
    - Max tracking error            (Section 9)
    - Control effort chatter        (Section 9, std of consecutive u deltas)
    - Mean |u| and max |u|          (control effort magnitude)
    - MPC only: velocity constraint violated? (bool) and peak |theta_dot|
      (only meaningful for MPC, which has an explicit constraint; PID/FF
      have no such constraint to violate by design)

## Statistical Aggregation

For each (controller x condition) pair, across its 50 trials, report:

    - Mean and standard deviation of RMS error
    - Mean and standard deviation of max error
    - Mean and standard deviation of control chatter
    - For MPC: constraint violation RATE (fraction of 50 trials where
      the velocity limit was breached at all)

This produces a 3 (controllers) x 4 (conditions) x ~6 (metrics, mean+std
pairs) results table - the core data for Task 20's comparison plots and
the eventual paper's Results section.

## Explicit Non-Goals (Scoping Honesty)

This benchmark does NOT re-run Stage 3/4's RTOS/safety mechanisms
(watchdog, sensor voting, active-hold) - those were validated separately
via Zephyr fault injection with their own dedicated evidence (DEVLOG
Stage 3-4). This benchmark is specifically a CONTROLLER comparison
(tracking performance, effort, constraint handling) under the Stage 1/2
Python simulation environment, not a full-system RTOS benchmark. Mixing
the two would conflate two genuinely different kinds of evidence.

## Implementation Plan (for Tasks 18-19)

    stage5_benchmarking/
      TEST_PROTOCOL.md          <- this file
      run_benchmark.py          <- Task 18: instrumentation + single-trial runner
      benchmark_results.csv     <- Task 19: raw output, all 600 trials
      benchmark_summary.csv     <- Task 19: aggregated mean/std per condition
      *.png                     <- Task 20: comparison plots
