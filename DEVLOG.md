
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
