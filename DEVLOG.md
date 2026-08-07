
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
