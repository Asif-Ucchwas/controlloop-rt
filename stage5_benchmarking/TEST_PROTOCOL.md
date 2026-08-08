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

Sinusoidal reference: amplitude=0.75 rad, frequency=0.5 Hz, duration=5.0s.
(Amplitude reduced from Stage 1/2's original 1.0 rad during Task 18's
smoke test - see note below.)
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

## Honest Limitation: Control Loop Rate Differs by Controller

Corrected smoke test (0.75 rad amplitude) still shows MPC's max_error
and settling_time notably worse than FF's, despite adequate constraint
margin. Root cause, verified: PID/FF run at 1kHz (1ms period, matching
Stage 1), MPC runs at 50Hz (20ms period, matching Stage 2 Task 7's real
solve-time constraint of mean~14ms/max~32ms per step). A 20x coarser
update rate inherently produces larger peak transient error and slower
settling against a moving reference, independent of the underlying
algorithm's quality.

Unlike the amplitude issue above, this is NOT something to equalize -
Stage 2 already established that MPC cannot run faster than ~50Hz on
this hardware without missing its own control deadline. Documented here
as an honest, inherent difference in HOW each controller operates
(discretization rate), not a flaw to fix. The Results section should
report this explicitly alongside the RMS/max-error/settling numbers, so
"MPC has best RMS but worst settling" reads as an explained tradeoff,
not an unexplained inconsistency.

## Implementation Plan (for Tasks 18-19)

    stage5_benchmarking/
      TEST_PROTOCOL.md          <- this file
      run_benchmark.py          <- Task 18: instrumentation + single-trial runner
      benchmark_results.csv     <- Task 19: raw output, all 600 trials
      benchmark_summary.csv     <- Task 19: aggregated mean/std per condition
      *.png                     <- Task 20: comparison plots

---

## Task 18 Addendum: Trajectory Amplitude Correction

Smoke-testing all 3 controllers (before committing to the full 600-trial
run) surfaced a real design flaw: the original amplitude=1.0 rad
trajectory requires a peak velocity of 2*pi*0.5*1.0 = pi ~= 3.14 rad/s -
which EXCEEDS MPC's own velocity constraint (+/-3.0 rad/s, reused from
Stage 2). This made the trajectory technically infeasible for MPC to
track while imposing no equivalent limit on PID/FF, which undermines
the "identical test conditions" goal this protocol is built around:
it's not a fair 3-way comparison if one controller is structurally
prevented from attempting what the other two can freely chase.

There is also a statistical argument, not just a fairness one: this
constraint-edge lag is deterministic, not random - it would appear
identically in all 50 MPC trials per condition, adding no genuine
statistical insight while confounding the mean/std comparison.

MPC's constraint-tradeoff behavior (deliberately sacrificing tracking
speed for guaranteed limit satisfaction) is already rigorously
documented in Stage 2 (Tasks 6-7) - this benchmark does not need to
rediscover that finding by accident.

**Fix:** amplitude reduced to 0.75 rad (frequency unchanged at 0.5 Hz,
matching Stages 1-2's validated timing), giving a peak velocity of
2*pi*0.5*0.75 ~= 2.36 rad/s - about 21% margin under MPC's constraint,
so all three controllers can genuinely attempt the full trajectory.

## Task 18: Metric Definitions & Honest Scoping

### Tracking error (RMS, max) - Section 9 of the math reference doc

Directly applicable to a sinusoidal reference as-is - no redefinition
needed. Computed per-trial for all 3 controllers under all 4 conditions.

### Control effort (mean |u|, max |u|, chatter) - Section 9

Directly applicable as-is. Computed per-trial for all 3 controllers.

### Settling time - REDEFINED for a sinusoidal reference

Section 9's original definition (2% band around a fixed target) does
not apply here - there is no single constant value to settle at. New
definition for this benchmark:

    steady_state_rms = RMS tracking error over the FINAL 20% of the trial
                        (t in [4.0s, 5.0s] of the 5.0s run)

    settling_time = first time t such that, for all t' >= t, the
                    instantaneous |error(t')| stays within 1.5x
                    steady_state_rms

Interpretation: time for the initial transient (starting from theta=0,
theta_dot=0, chasing an already-moving sinusoid) to decay into the
trajectory's eventual steady periodic tracking pattern. This is an
adapted, not identical, metric to Section 9's step-response definition -
documented explicitly as a redefinition, not a silent reuse.

### Watchdog trigger rate / task timing jitter - SCOPING NOTE

These metrics have REAL measured data ONLY for PD, from Stage 3-4's
actual Zephyr RTOS port (DEVLOG Stage 3 Task 12: zero jitter under
stress-tested load; Stage 4 Task 13: watchdog detection latency ~3.5-4ms).

Feedforward was never ported to the RTOS - no jitter/watchdog data
exists for it, and none will be fabricated for this benchmark.

MPC's RTOS-readiness has only an INDIRECT proxy: Stage 2 Task 7 measured
MPC's Python solve time (max 32.84ms at N=20) against the 20ms control
period a real RTOS port would need - a genuine, measured deadline-miss
RISK, but never actually run under a real watchdog in Zephyr. Task 18-20
report this as "MPC solve-time deadline margin" (a Python-measured
proxy), explicitly NOT as "MPC watchdog trigger rate" (which would imply
data that doesn't exist).

**Summary table for the eventual paper's Results section:**

    | Metric                  | PID/PD | PD+FF | MPC                        |
    |--------------------------|--------|-------|------------------------------|
    | Tracking error (RMS/max) | Yes    | Yes   | Yes  (this benchmark)        |
    | Control effort           | Yes    | Yes   | Yes  (this benchmark)        |
    | Settling time (adapted)  | Yes    | Yes   | Yes  (this benchmark)        |
    | RTOS jitter              | Yes    | No    | No   (Stage 3, PD only)      |
    | Watchdog detection       | Yes    | No    | No   (Stage 4, PD only)      |
    | Deadline-miss risk       | N/A    | N/A   | Yes  (Stage 2 Task 7, proxy) |
