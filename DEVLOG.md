
## Stage 1 — Plant Modeling & Baseline Control (Tasks 1-4)

- Built DC servo motor plant model (2nd-order, inductance neglected) — verified
  open-loop step response matches analytical steady-state slope (0.0999 rad/s).
- First PID attempt (Kp=20, Ki=40, Kd=0.5) produced 54% overshoot and failed
  to settle within 3s. Diagnosed via closed-loop pole computation: dominant
  poles at -0.88±j1.95 (ζ≈0.41), a genuinely underdamped, slowly-decaying mode
  — not a simulation bug.
- Root cause: plant transfer function already has an integrator built in
  (1/(s²+10.01s)); adding Ki created a redundant second integrator. Switched
  to PD-only, using pole placement (ζ=0.7, ωₙ=12) to derive Kp=144, Kd=6.79.
  Verified actual closed-loop poles matched predicted poles almost exactly
  (-8.4±j8.57 both ways).
- Result: 6.06% overshoot, 0.447s settling time, zero steady-state error.
- Added feedforward (model-inversion) on top of PD for sinusoidal reference
  tracking. 87.4% RMS tracking error reduction vs. feedback-only. Documented
  the honest limitation: feedforward doesn't help on step inputs (plant
  already has an integrator), and residual max error is an initial-condition
  transient, not a modeling gap.
- See docs/notes/plant_model_and_baseline.md for full derivation and results.

## Stage 2 Task 5 — MPC Formulation

- Set up do-mpc with n_horizon=20, t_step=0.02 (50Hz — realistic for a
  QP-solving control loop, much slower than PID's 1kHz since MPC solves an
  optimization problem every step, not simple arithmetic).
- Unconstrained step response (voltage bounds ±100V, essentially non-binding):
  8.18% overshoot, 0.94s settling, ~0 steady-state error.
- Honest finding: Task 2's pole-placed PD baseline (6.06% overshoot, 0.447s)
  outperforms this MPC configuration on the same unconstrained step test.
  This is expected, not a bug — MPC's real advantage is constraint handling,
  not raw unconstrained tracking speed against a well-tuned classical
  controller. Task 6 tightens the voltage bounds to demonstrate the actual
  case for MPC.

## Stage 2 Task 6 — Hard Constraints

- Actuator saturation only (V_LIMIT=5V), PD with naive post-hoc clamping:
  PD actually matched or slightly beat MPC (0.42% vs 1.30% overshoot,
  near-identical ~2.1s settling). Diagnosed why: our PD baseline has no
  integral term, so there's no windup to punish — once saturated, PD
  effectively behaves like a bang-bang controller anyway, which is close
  to what a constrained-optimal solution looks like for pure actuator
  limits. Actuator saturation alone, on an I-free baseline, doesn't
  showcase MPC's real advantage.
- State constraint (velocity limit, θ̇ ≤ 3.0 rad/s) — the real differentiator:
  - PD peak |θ̇|: 5.438 rad/s — VIOLATED the limit by 81%, with no
    structural mechanism to prevent it (PID has no notion of a velocity
    constraint; the only lever is indirect gain re-tuning).
  - MPC peak |θ̇|: 2.883 rad/s — respected the limit by construction, since
    it's an explicit bound in the QP, not an emergent property of gains.
- Conclusion for the paper/interview narrative: MPC's advantage over
  classical control isn't raw unconstrained tracking speed (PD wins there)
  — it's the ability to enforce state constraints directly and reliably,
  something PID/PD has no mechanism to do at all. This is a more precise
  and more honest claim than "MPC is better," and it's the one the data
  actually supports.

## Stage 2 Task 7 — Horizon Tuning & Benchmark vs. Stage 1 Baseline

Swept n_horizon = [5, 10, 20, 40] at t_step=0.02s, same velocity-constrained
scenario as Task 6.

| N  | Settling(s) | Overshoot% | Peak θ̇ | Mean solve(ms) | Max solve(ms) |
|----|-------------|------------|---------|----------------|---------------|
| 5  | never       | (unstable) | 2.184   | 10.03          | 13.21         |
| 10 | never       | 63.07%     | 3.000   | 12.20          | 33.19         |
| 20 | 0.96s       | 7.97%      | 2.883   | 14.33          | 32.84         |
| 40 | 0.96s       | 5.79%      | 2.785   | 22.92          | 29.36         |

**N=5 diagnosis:** the raw -5138% overshoot number was a division artifact
(near-zero denominator), not the real finding. Direct trace of theta showed
a genuine large-amplitude, slow oscillation: theta=1.31 at t=1s, 1.74 at
t=2s (still rising), -0.04 at t=3s and still falling — never converging
within the test window. Root cause: horizon length (5 x 0.02s = 0.1s) is
shorter than the plant's own closed-loop time constant (~0.12s, from Stage
1's poles at -8.4+/-j8.57). MPC cannot "see" the consequence of its control
action before committing to it. This demonstrates prediction horizon is a
STABILITY requirement for MPC, not merely a performance tuning knob —
worth stating explicitly in the paper's methodology section.

**N=10 and N=20/40 comparison:** N=10 settles faster nominally but 63%
overshoot with the velocity constraint pinned at its exact limit (3.000 —
suspicious of active constraint clipping, not a converged optimum). N=20
and N=40 both settle at 0.96s; N=40 buys a modest overshoot improvement
(7.97% -> 5.79%) at a real cost: mean solve time nearly doubles
(14.33ms -> 22.92ms).

**Chosen horizon: N=20.** Reasoning: N=40's overshoot improvement is
marginal relative to its ~60% increase in mean solve time — and solve
time is the number that directly threatens the real-time budget once this
runs on Stage 3's RTOS.

**Real-time deadline finding (important for Stage 3/4):** N=20's
max_solve_ms = 32.84ms exceeds the 20ms control period (t_step=0.02s) it
needs to fit inside. This is a genuine, measured real-time deadline MISS
in worst case, not a hypothetical concern — directly relevant to Stage 4's
watchdog design, which needs to handle exactly this failure mode (a
control step that doesn't finish in time). Will need to either: reduce
solver iterations, use a warm-start strategy, or design the watchdog
fallback assuming occasional overruns are expected, not exceptional.

## Comparison vs. Stage 1 Baseline (unconstrained step, per Task 5)

| Controller | Overshoot | Settling |
|---|---|---|
| PD (Stage 1, pole-placed) | 6.06% | 0.447s |
| MPC (N=20, unconstrained, Task 5) | 8.18% | 0.94s |

PD baseline remains faster and lower-overshoot on the unconstrained case
even after horizon tuning. This reinforces Task 6's conclusion: MPC's
value in this project is constraint enforcement (Task 6's velocity limit
result), not raw unconstrained tracking speed. N=20 is retained as the
project's standard MPC configuration for Stage 5 benchmarking, chosen
specifically for its solve-time margin, not because it beat PD on speed.

## Stage 2 Task 8 — Stress Test: Sensor Noise + Unmodeled Disturbance

Injected realistic sensor noise (theta: std=0.01 rad, theta_dot: std=0.05
rad/s, applied every measurement) plus a genuinely unmodeled disturbance
(extra -80 rad/s^2 torque-equivalent term active t=[1.0, 1.3]s, present
in the true plant dynamics but NOT in either controller's model). Same
random seed for both controllers for a fair comparison.

| Controller | Settling | Peak θ̇ | Constraint violated? | Final error | Control chatter (std of ΔV) |
|---|---|---|---|---|---|
| PD  | 1.966s | 4.470 rad/s | Yes, 49.0% over | 0.0100 rad | 115.389 |
| MPC | 2.16s  | 3.912 rad/s | Yes, 30.4% over | 0.0000 rad | 8.229 |

**Finding 1 (important honest caveat to Task 6's claim):** MPC's velocity
constraint, which held exactly under ideal conditions in Task 6, was
VIOLATED here too under a real, unmodeled disturbance. This is expected,
not a bug — it's the textbook limitation of *nominal* MPC: the QP's
constraint guarantee only holds relative to the controller's internal
model. Since the disturbance isn't in model.set_rhs, MPC's predictions
during t=1.0-1.3s are simply wrong, and the "enforced" constraint slips.
This is precisely the gap that robust/tube MPC variants are designed to
close (explicitly bounding disturbance uncertainty in the optimization) -
named here as a scoping honesty note, same convention as the ISO 26262
"documented, not certified" framing planned for Stage 4.

**Finding 2:** despite both violating the constraint, MPC still degraded
more gracefully - 30.4% margin vs. PD's 49.0%. Even disturbance-unaware,
MPC's optimization structure absorbed the shock better than reactive PD.

**Finding 3 (the strongest, most practically significant result of Stage 2):**
control effort chatter. PD's control signal derivative had std=115.389,
MPC's had std=8.229 - a ~14x difference. Root cause: PD's D-term directly
differentiates the raw noisy position measurement, and differentiation
always amplifies noise. MPC never explicitly differentiates anything - it
solves an optimization over the full noisy state with an explicit
smoothness penalty (rterm=0.01*V^2) built in, which acts as an implicit
filter. This is arguably MPC's single most practically relevant advantage
on this plant: 115V/step chatter would be genuinely damaging or trip
overcurrent protection on a real motor driver; 8V/step is a reasonable,
usable control signal.

## Stage 2 Complete - Summary for Paper/Interview Narrative

Four honest findings across Tasks 5-8, none of which is a simple "MPC wins":
1. Unconstrained tracking: PD wins (Task 5) - MPC's overhead buys nothing
   here on this simple linear plant.
2. Actuator saturation alone: roughly a tie (Task 6) - no windup to punish
   without an I-term, so naive PD clamping is nearly as good as MPC.
3. State (velocity) constraint, ideal conditions: MPC wins decisively
   (Task 6) - PD has no mechanism to enforce this at all.
4. State constraint under real disturbance: MPC's guarantee degrades but
   still outperforms PD (Task 8) - and MPC's noise-rejection/control-
   smoothness advantage (14x lower chatter) is the standout practical win.

Overall narrative: MPC's value on this plant isn't raw tracking speed -
it's (a) explicit state constraint handling under nominal conditions, and
(b) dramatically better control-effort smoothness under sensor noise.
Both are real, measured, and honestly scoped, including where MPC's
guarantees break down (unmodeled disturbances) - a more credible and
interview-defensible story than an uncomplicated "MPC beats PID" claim.

## Stage 3 Task 9 — Reuse CAN-Net Zephyr/QEMU Infrastructure

Created a new out-of-tree Zephyr application (stage3_rtos/control_loop_rt/)
that builds against the SAME shared west workspace as CAN-Net
(~/projects/can-net/zephyr_project/), avoiding a second ~1GB+ SDK/module
download. Key mechanism: CMakeLists.txt uses
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE}), and west build's
-s (source/app dir) and -d (build dir) flags let it build an app living
entirely outside the workspace tree, while west resolves ZEPHYR_BASE
internally by locating the workspace's .west config (found by running
west from inside the CAN-Net zephyr_project directory).

Verified with a minimal printk-only main.c: clean build (100/100), ran
under native_sim, correct boot banner and output confirmed. No
ZEPHYR_BASE export needed in .bashrc - west handles this per-invocation
based on where it's run from and the -s/-d flags.

Reused: zephyr_venv (Python venv with west), zephyr-sdk-1.0.1 (SDK,
globally installed), zephyr_project/zephyr (Zephyr 4.4.99 source tree,
same version CAN-Net used).

## Stage 3 Task 10 — Port PD Control Loop into Periodic Zephyr Task

Ported Stage 1's PD controller (Kp=144, Kd=6.79) and DC servo plant model
into a C-based periodic Zephyr task (stage3_rtos/control_loop_rt/),
running under native_sim, using k_timer + k_sem for fixed-rate 1kHz
scheduling instead of a Python for-loop.

Two real bugs found and fixed on first run:

**Bug 1 - Derivative kick (144x voltage spike at t=0):** the C port
initially differentiated ERROR (error - prev_error)/dt, same as Stage 1's
Python script. At t=0, error jumps instantly from 0 to 1.0 (target minus
initial theta=0), producing a huge fake derivative (u=6934V on the first
step). This is a known control-engineering issue called "derivative
kick" - almost certainly present in Stage 1's Python PD script too, just
never surfaced since we never printed/checked the first-step voltage
there. Fixed with the standard technique: derivative-on-measurement,
differentiating theta directly (-(theta-prev_theta)/dt) instead of
error, since the plant state changes smoothly even when the target
doesn't. Confirmed fix: first-step u=144.00V (pure P-term, as expected
since prev_theta=theta=0 initially).

**Bug 2 - Silent timing mismatch (requested 1ms, got 10ms):**
native_sim's default CONFIG_SYS_CLOCK_TICKS_PER_SEC=100 means the RTOS
can only resolve time in 10ms ticks. Our K_MSEC(1) request was silently
rounded up to the nearest tick with no warning - the loop was actually
running at 100Hz, not the requested 1kHz, and nothing in the build or
run output flagged this. Found by explicitly measuring and printing
actual inter-wake periods rather than trusting the requested rate. Fixed
by raising CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000 (0.1ms tick resolution)
in prj.conf. Confirmed fix: measured periods now read exactly 1000us
(1ms) consistently.

**Important honest caveat for the timing claim:** native_sim's clock is
a deterministic simulated clock, not a model of real hardware interrupt
latency, cache effects, or OS scheduling jitter. The "perfect" 1000us/
1000us min/max period seen here reflects native_sim's idealized timing
model, NOT proof of zero real-world jitter. Genuine timing variability
will only appear once a second competing task is added (Task 11) or on
real hardware (optional Stage 7) - this distinction matters for the
paper's Methodology/Limitations section.

Result: theta converges to 1.0000 by t=1.0s, matching Stage 1's Python
PD baseline (0.447s settling, zero steady-state error) - confirms the C
port is behaviorally equivalent to the validated Python model, not just
"it compiles."

Reused CAN-Net's zephyr_project workspace and zephyr_venv per Task 9 -
no duplicate SDK/module download.

## Stage 3 Task 11 — Second Task, Demonstrate Preemption

Added logging_task (priority 7, lower priority than control_task's
priority 5 - lower number = higher priority in Zephyr) doing ~2ms of
simulated busy-work per cycle, deliberately longer than control_task's
1ms period, specifically to test whether preemption actually holds under
real competition.

**Bug found on first run - full hang, no output after both tasks'
startup messages.** Root cause: logging_task used a manual busy-wait
loop checking k_uptime_get() against a target duration. native_sim's
simulated clock only advances on kernel scheduling events - a tight loop
making no kernel calls never yields, so simulated time never moved, and
the loop spun forever in zero simulated time, starving control_task
entirely. This is the EXACT gotcha already documented in CAN-Net's
DEVLOG/memory ("native_sim's virtual clock only advances on kernel
scheduling events, busy-waits must use k_busy_wait() not manual
k_uptime_get() spin loops") - reused the Zephyr infrastructure per Task
9, but the specific lesson didn't automatically carry into new code.
Worth the reminder: documented gotchas need to be actively re-applied,
not just recorded once. Fixed by replacing the manual loop with
k_busy_wait(2000) - the correct primitive, which properly advances
simulated time while still allowing async interrupts to be delivered.

**Result after fix:** control_task's period measured EXACTLY 1000us
(min=max=avg=1000us) across all 2999 cycles, despite logging_task
genuinely competing with 2ms of busy-work every cycle. Confirms real,
correct priority-based preemption: the timer ISR fires every 1ms, wakes
control_task via k_sem_give(), and the scheduler preempts logging_task
immediately on ISR return - even mid-busy-wait - because k_busy_wait()
does not disable interrupts, only avoids voluntary yielding.

**Honest caveat (same as Task 10, worth restating not dropping):** the
zero measured variance is native_sim's idealized deterministic clock
proving the SCHEDULER's logical correctness (priorities are honored
exactly as designed) - it is not evidence that real hardware would show
zero jitter under equivalent load. Real interrupt latency, cache
effects, and bus contention would introduce measurable variance even
with functionally correct preemption. This distinction is important for
the paper's Methodology/Limitations section, same framing as Task 10.

## Stage 3 Task 12 — Timing Jitter Measurement Under Load

Before trusting Task 11's "zero jitter" result, checked whether it was
real or just measurement blindness: Task 11 measured timing via
k_uptime_ticks() at CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000 (100us
resolution) - meaning any jitter smaller than 100us would have been
invisible. Upgraded to k_cycle_get_32() at CONFIG_SYS_CLOCK_HW_CYCLES_
PER_SEC=1000000 (1us resolution, 100x finer), AND increased the
competing load's difficulty: logging_task now does randomized 1000-4000us
busy-work per cycle (was a fixed 2000us in Task 11) to actually stress-test
for a breaking point rather than re-confirm the easy case.

**Result: still exactly zero jitter.** min=1000us, max=1000us,
avg=1000us, jitter(max-min)=0us, deadline misses=0/2999 (0.00%) - even
at 100x finer measurement resolution and under harder, randomized
competing load up to 4x the control period's own length.

**Precise conclusion (stronger than Task 10/11's caveat, now backed by
an actual attempt to break it):** native_sim's simulation model
structurally EXCLUDES the physical sources of real timing jitter -
silicon-level interrupt latency, cache misses, pipeline flushes, bus
contention. No amount of scheduling-level stress can surface jitter that
the simulation doesn't model in the first place. This is a stronger,
more specific claim than "native_sim is idealized": we didn't just
assume it, we tried to break it with a harder test and confirmed the
simulation has no jitter SOURCE to find, regardless of competing load.
Genuine jitter numbers require real hardware - this is the concrete,
specific justification for Stage 7 hardware validation, not a vague
disclaimer.

**Jitter metric definition used:** jitter = max_period - min_period,
over N measured control-loop periods. (Note for math reference doc: add
alongside RMS/max-error/chatter/settling-time definitions in Section 9,
since this is another metric computed in code but not formally defined
until now.)

## Stage 3 Complete (Tasks 9-12)

Summary: reused CAN-Net's Zephyr/QEMU infrastructure with zero duplicate
download (Task 9); ported the Stage 1 PD control loop into a real 1kHz
periodic Zephyr task, finding and fixing two genuine bugs along the way
(derivative kick, silent tick-rate mismatch - Task 10); added a
competing lower-priority task and proved real interrupt-driven
preemption, catching and fixing a native_sim clock-model hang in the
process (Task 11); and confirmed under a deliberately harder stress test
that native_sim's zero-jitter result is a real property of the
simulation model, not measurement blindness (Task 12).

Four real bugs found and fixed across this stage, each documented with
root cause rather than just "fixed it" - genuine debugging material for
interviews, and a coherent methodology narrative for the eventual paper:
scheduling correctness is proven in simulation; physical timing jitter
is explicitly named as requiring real hardware (Stage 7), not
overclaimed as already measured.

## Stage 4 Task 13 — Software Watchdog with Fault Injection

Implemented an independent watchdog thread (priority 3, higher than
control_task's priority 5, so the monitor cannot be starved by whatever
it's monitoring) that expects a "kick" from control_task every cycle. If
no kick arrives within 2500us (checked every 2ms), it triggers a
fallback: force control output to 0V and latch that state.

**Fault injection method:** rather than simulating a genuine infinite
loop (which would hang the whole process), deliberately withheld the
kick for 10 consecutive control cycles - this tests the actual symptom
the watchdog monitors (missed check-ins), which is the correct and
standard way to unit-test a watchdog's detection path.

**Test 1 (easy case) - fault injected at step 1500, after settling:**
watchdog detected the fault in ~3.5-4ms (bounded by timeout 2500us +
polling period 2000us, matching the theoretical worst case). Fallback
correctly zeroed voltage. theta stayed exactly at 1.0000 with no visible
change, since the system was already at rest (theta_dot~=0) when the
fault hit - power cutoff had nothing to counteract.

**Test 2 (hard case, more rigorous) - fault injected at step 50, mid-
transient, while theta was still climbing under ~90-120V:** watchdog
detected the fault in ~4ms (step 54, consistent with Test 1's latency
bound). BUT: theta CONTINUED RISING after the power cutoff - from
0.1579 rad at detection to a final resting value of 0.6348 rad, a coast
of ~0.48 rad (~27 degrees) that took roughly 200-300ms to fully decay.

**Critical finding: cutting power is NOT equivalent to an instant stop.**
Root cause: theta_dot (velocity) doesn't vanish when voltage is cut -
it only decays through the plant's own friction term, with time constant
1/|A21| = 1/10.01 ~= 100ms (same number from Stage 1's pole analysis).
A naive "kill power" fallback lets the system coast on inertia for
multiple hundred-millisecond time constants before actually stopping.

This is a genuine, non-obvious safety-engineering lesson (the same
principle behind real braking-system design: de-energizing an actuator
does not stop a moving mass) and directly motivates Task 15's design:
a naive power-cutoff fallback can itself be unsafe if coast distance
matters for the application (e.g. a robot joint near an obstacle).
Task 15 will need a smarter safe-state response - likely actively
holding the CURRENT position (re-targeting the PD loop to theta-at-
fault-time) rather than simply cutting power to zero.

**Metric introduced: coast distance under power-cut fault** = 
|theta_final_after_coast - theta_at_fault_detection|. Measured: 0.48 rad
for Test 2's mid-transient fault. Will add formal definition to math
reference doc alongside jitter/RMS/chatter definitions.

Kept the mid-transient (Test 2, step 50) fault injection as the
project's canonical/default test scenario in the committed code, since
it's the more rigorous case and produces the more important finding.

## Stage 4 Task 14 — Dual-Sensor Redundancy & Voting

Significant realism upgrade: control_task no longer reads ground-truth
theta directly (as it had in every prior task) - it now goes through two
independent simulated sensor channels (each with small Gaussian-like
noise, amplitude 0.005 rad, via a simple LCG-based pseudo-random
generator) and a voting layer. This is a meaningfully more realistic
architecture than Stages 1-3's "perfect state knowledge" assumption.

**Voting logic:** if |sensor_A - sensor_B| < 0.05 rad, trust their
average. If they disagree beyond that, fall back to sensor A (designated
primary) and flag a disagreement - not an immediate shutdown; Task 15
will build the fuller escalation logic on top of this.

**Fault injection:** biased sensor B by +0.5 rad for 200 cycles
(steps 800-999), simulating a stuck/miscalibrated encoder, while sensor A
continued reading correctly.

**Result: voting logic performed perfectly.**
- disagreement_count = 200/3000, exactly matching the injected fault
  duration - zero missed detections, zero false positives.
- Quantified the cost of NOT voting: naive_avg_would_be sat at ~1.245-
  1.253 throughout the fault (vs. true_theta ~0.998-1.000) - a
  persistent, sustained ~0.25 rad (~14 degree) FALSE reading that a
  naive averaging approach would have fed into the control law for the
  entire 200ms fault window.
- Most important result: true_theta stayed essentially flat (0.998-
  1.000) throughout the fault window. The voting logic didn't just
  detect the bad sensor - it fully protected the physical plant from any
  disturbance, because the corrupted channel was correctly excluded
  before ever reaching the control law.

**Benign artifact, not a bug:** a watchdog fault fired AFTER
"[CONTROL] DONE." at the very end of the run. This happens because
control_task's while loop legitimately exits at step 3000, the thread
function returns, and kicks simply stop - which the watchdog correctly
cannot distinguish from a real hang (and shouldn't be able to: in a real
embedded system, control_task should never return at all, so any
cessation of kicks legitimately indicates a fault). Documented as
expected test-harness behavior.

**Architectural note:** this task demonstrates two independent fault-
detection mechanisms (Task 13's watchdog for scheduling/liveness faults,
Task 14's voting for sensor-data faults) coexisting and triggering the
same underlying safety infrastructure (control_output_enabled flag) -
a modular safety-layer design, not a single monolithic fault check.
