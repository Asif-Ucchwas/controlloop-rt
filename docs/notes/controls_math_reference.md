# Controls Math Reference — Interview Prep

Formula → derivation → where it's implemented. Extended as each stage adds
new math (MPC's QP formulation, RTOS jitter statistics, etc.).

---

## 1. DC Motor Plant Derivation

### Physical equations (before simplification)

Two coupled subsystems:

**Electrical (Kirchhoff's voltage law around the armature loop):**
`V` = applied voltage, `i` = armature current, `L` = inductance, `R` =
resistance, `K·θ̇` = back-EMF (motor generates a voltage opposing its own
spin, proportional to speed).

**Mechanical (Newton's law for rotation, torque = J·angular acceleration):**
`J` = rotor inertia, `b` = viscous friction (drag proportional to speed),
`K·i` = motor torque (proportional to current).

### Simplification: neglect inductance

Electrical dynamics settle on the order of milliseconds; mechanical
dynamics settle on the order of tenths of a second. So we assume the
electrical loop is always at steady state (`di/dt ≈ 0`):
Substitute into the mechanical equation:
Note the `K²/R` term: this is "electrical damping" — the motor's own
back-EMF acts like extra mechanical friction. It's a real physical effect,
not a modeling artifact.

### State-space form

Let `x1 = θ` (position), `x2 = θ̇` (velocity). Then `ẋ1 = x2` by
definition, and `ẋ2 = θ̈` comes straight from the ODE above:
In matrix form `ẋ = Ax + Bu`, `y = Cx`:
**Worked numbers (our system):** J=0.01, b=0.1, K=0.01, R=1.0
Matches exactly what `control.ss()` printed.

**Code:** `stage1_plant/dc_servo_model.py`

---

## 2. State-Space → Transfer Function

For a SISO system, the transfer function is:
For our A, B, C, D (D=0), this works out to:
**Why the bare `s` in the denominator matters:** a transfer function's pole
at `s=0` means the system contains a free integrator. Physically: this is
a position system driven by a velocity-proportional input path — apply any
constant voltage and position grows without bound (it never "settles" to a
value on its own). This single fact is what drove the Task 2 PID failure
and fix below — worth being able to explain out loud.

**Code:** `stage1_plant/dc_servo_model.py` (`ct.ss2tf(sys)`)

---

## 3. Why Ki Broke the Loop (System Type / Final Value Theorem)

**System Type** = number of free integrators (poles at s=0) in the
*open-loop* transfer function. Our plant is already **Type 1**.

**Final Value Theorem** tells us the steady-state error to a step input for
a Type N system with feedback gain C(s):

- Type 0 (no integrator anywhere): finite steady-state error — never
  reaches the target exactly.
- Type 1 (one integrator, ours or the controller's): **zero** steady-state
  error to a step, but nonzero to a ramp.
- Type 2 (two integrators): zero error to both step AND ramp, but now
  you've got extra phase lag, which is what caused our oscillation.

**Our mistake:** plant is already Type 1 (from its own `1/s` factor). Adding
`Ki/s` in the PID controller made the *open-loop* system Type 2. Type 2
systems are much more prone to poor damping unless very carefully tuned —
which is exactly the 54% overshoot, non-settling oscillation we measured.

**The fix:** since the plant already provides Type 1 behavior, the
controller only needs P and D — no integral term required for zero
steady-state error on a step target.

**Code:** the diagnostic pole computation in `stage1_plant/pid_baseline.py`

---

## 4. Pole Placement Design (PD Gains)

**Closed-loop characteristic equation.** For plant `G(s) = 1/(s² + as)`
(where `a = (b+K²/R)/J = 10.01`) with PD controller `C(s) = Kp + Kd·s`,
the closed-loop transfer function is:
The denominator of `T(s)` — the closed-loop characteristic equation — is:
**Standard 2nd-order form.** Every 2nd-order system's behavior (overshoot,
settling time, oscillation) is fully described by two numbers: damping
ratio `ζ` and natural frequency `ωₙ`, via the standard form:
**Choosing ζ and ωₙ** (the actual design decision, not just algebra):
- `ζ = 0.7` is the classic "good enough" choice — near-critically-damped,
  gives ~4.6% overshoot in theory (textbook formula below), fast without
  ringing.
- `ωₙ = 12 rad/s` was chosen to get sub-second settling without demanding
  unrealistic control effort (higher ωₙ = faster response but bigger
  voltage swings).

**Worked numbers:**
Matches exactly what we used.

**Verifying via poles.** Roots of `s² + 2ζωₙs + ωₙ² = 0`:
Matches the actual computed poles (`-8.4 ± j8.57`) exactly — confirms the
design math and the simulation agree.

**Standard 2nd-order performance formulas** (useful for quick estimates
without simulating):
Plug in ζ=0.7, ωₙ=12:
Close to measured (6.06% overshoot, 0.447s) — small gap is expected since
these formulas assume a pure 2nd-order system with no zeros; our simulation
is the real (very slightly different) discretized system.

**Code:** `stage1_plant/pid_baseline.py`

---

## 5. Feedforward Control (Model Inversion)

**Core idea:** if you know the plant's dynamics exactly, you can compute
in advance the exact input needed to produce a desired trajectory — instead
of waiting for feedback to notice an error and react to it.

**Derivation.** Start from the plant ODE:
Solve for V given a *desired* trajectory `θ_ref(t)` (i.e., substitute the
reference's position/velocity/acceleration in place of the plant's actual
state):
This is literally "solve the plant equation backwards" — given where you
want to be, what voltage produces exactly that motion (in the idealized,
no-disturbance case).

**Why it doesn't help on a step input:** for a step, `θ̇_ref = 0` and
`θ̈_ref = 0` everywhere except the instant of the step itself (technically
an impulse in acceleration, undefined for a real system) — so `V_ff ≈ 0`
almost everywhere. Feedforward has nothing useful to contribute when the
target isn't moving. This is why we tested it on a sinusoid instead — a
genuinely moving reference.

**Why feedback is still needed even with feedforward:** feedforward is
only as good as the model. Any mismatch between the model (J, b, K, R) and
the real plant, or any external disturbance, isn't corrected by
feedforward at all — that's still feedback's job. This is why the combined
result (feedback+feedforward) beat pure feedforward-only.

**Code:** `stage1_plant/feedforward_comparison.py`

---

## Quick-Reference Formula Sheet

| Concept | Formula |
|---|---|
| DC motor simplified ODE | `J·θ̈ + (b+K²/R)·θ̇ = (K/R)·V` |
| State-space A[1,1] | `-(b+K²/R)/J` |
| State-space B[1,0] | `K/(R·J)` |
| 2nd-order standard form | `s² + 2ζωₙs + ωₙ² = 0` |
| Pole placement: Kp | `ωₙ²` |
| Pole placement: Kd | `2ζωₙ - a` (a = plant's own damping term) |
| Closed-loop poles | `-ζωₙ ± jωₙ√(1-ζ²)` |
| % Overshoot (2nd order) | `100 × exp(-ζπ/√(1-ζ²))` |
| Settling time (2% band) | `≈ 4/(ζωₙ)` |
| Feedforward voltage | `V_ff = (R/K)[J·θ̈_ref + (b+K²/R)·θ̇_ref]` |
| System Type | count of poles at s=0 in open-loop G(s) |

---

## 6. MPC Cost Function & Constraint Structure

**General MPC problem, solved fresh at every timestep:**
`N` = prediction horizon (we used 20 steps × 0.02s = 0.4s lookahead). `Q`,
`R`, `P` are weighting matrices (in our scalar case, just numbers): `Q`
penalizes position error, `R` (`set_rterm`) penalizes control effort.

**Receding horizon principle:** solve the above for the *entire* horizon,
but only apply `u_0` (the first control move). Re-measure state, re-solve
the whole problem again next timestep. This is what makes MPC naturally
handle disturbances and model mismatch — it's constantly re-planning from
the true current state, not committing to an open-loop plan.

**Why this structurally beats PID for state constraints:** PID computes
`u = f(error)` — a function only of the tracking error, with zero awareness
of any other state variable's limits. Adding "θ̇ must stay under X" to PID
requires either (a) indirect gain re-tuning with no guarantee, or (b) an
entirely separate outer-loop velocity limiter bolted on. MPC instead adds
one line — `x_min ≤ θ̇ ≤ x_max` — directly into the optimization the
controller already solves. The constraint is enforced by the solver, not
approximated by gain tuning.

**Measured result on our plant (θ̇ limited to ±3.0 rad/s):**
| Controller | Peak |θ̇| | Within limit? |
|---|---|---|
| PD (no mechanism to enforce limit) | 5.438 rad/s | No — 81% over |
| MPC (limit as explicit QP constraint) | 2.883 rad/s | Yes, by construction |

**Interview-ready one-liner:** "MPC's advantage isn't that it tracks faster
than a well-tuned PID — on my plant, PD actually settled faster on the
unconstrained case. MPC's advantage is that it can enforce state
constraints, like a velocity limit, directly and reliably — something PID
has no built-in mechanism to do at all."

---

## Worked Examples — Plugging In Real Numbers

This section exists specifically so you can re-derive every result above
from scratch, months later, without re-reading the whole reasoning trail.

### Worked Example A — State-space to transfer function (Section 2)

The formula: G(s) = C(sI - A)^-1 * B + D. With our numbers:

    A = [[0, 1], [0, -10.01]]
    B = [[0], [1.0]]
    C = [[1, 0]]
    D = 0

Step 1 -- form (sI - A):

    sI - A = [  s      -1   ]
             [  0    s+10.01]

Step 2 -- invert (2x2 inverse: swap diagonal, negate off-diagonal, divide
by determinant det = s(s+10.01)):

    (sI-A)^-1 = (1/det) [ s+10.01    1 ]
                         [    0      s ]

Step 3 -- multiply by B, then by C, to isolate the theta output:

    (sI-A)^-1 * B = (1/det) [ 1 ]
                             [ s ]

    C * (sI-A)^-1 * B = (1/det) * 1 = 1 / (s^2 + 10.01s)

Matches the printed transfer function exactly. This is the general
recipe -- for any A, B, C you compute, these three steps always work.

### Worked Example B — System Type check (Section 3)

Given any open-loop transfer function, count poles at s=0:

    Our plant alone: G(s) = 1/(s^2+10.01s) = 1/[s * (s+10.01)]
                            -> one factor of s in denominator -> Type 1

If you added a PI controller C(s) = Kp + Ki/s, the open-loop combo
C(s)*G(s) has denominator s*s*(s+10.01) = s^2*(s+10.01) -> Type 2.

Rule of thumb to memorize: count the s=0 poles in the combined
open-loop transfer function, not the plant alone, once a controller with
an integrator is added.

### Worked Example C — Feedforward voltage at a specific instant

Using the sinusoidal reference from Task 3 (amplitude 1 rad, freq 0.5 Hz),
evaluate the feedforward law at t = 0.5s:

    theta_ref(t)   = sin(2*pi*0.5*t)              -> theta_ref(0.5) = sin(pi*0.5) = 1.0
    theta_dot_ref  = 2*pi*0.5*cos(2*pi*0.5*t)      -> theta_dot_ref(0.5) = pi*cos(pi*0.5) ~= 0
    theta_ddot_ref = -(2*pi*0.5)^2*sin(2*pi*0.5*t) -> theta_ddot_ref(0.5) = -pi^2*sin(pi*0.5) ~= -9.87

    V_ff(0.5) = (R/K) * [J*theta_ddot_ref + (b+K^2/R)*theta_dot_ref]
              = (1.0/0.01) * [0.01*(-9.87) + 10.01*0]
              = 100 * (-0.0987)
              = -9.87 V

Interpretation: at t=0.5s the reference is at its peak (theta_ref=1.0) and
momentarily stationary (theta_dot_ref ~= 0) but about to accelerate hard
downward (theta_ddot_ref very negative, since it's about to swing back
toward zero) -- so feedforward correctly commands a strong negative
voltage to start pulling the motor back down, before feedback would even
notice the reference has stopped climbing.

### Worked Example D — MPC cost function with our actual numbers

Our do-mpc setup used n_horizon=20, t_step=0.02s, mterm=lterm=(theta-target)^2,
rterm=0.01*V^2. Plugging into the general formula:

    minimize  sum_{k=0}^{19} [ (theta_k - 1.0)^2 + 0.01*V_k^2 ]  +  (theta_20 - 1.0)^2

    subject to  theta_{k+1}, theta_dot_{k+1} from our A/B dynamics
                (Euler-discretized internally)
                -100 <= V_k <= 100          (Task 5/6 actuator bound)
                -3.0 <= theta_dot_k <= 3.0  (Task 6 velocity constraint)

Concretely: the horizon looks 20 * 0.02s = 0.4s into the future. At each
of those 20 predicted steps, being far from theta=1.0 costs (error)^2, and
commanding large voltage costs 0.01*V^2 -- the 0.01 weight means the
solver tolerates fairly large voltage swings before it's "worth it" to the
optimizer to trade tracking speed for smoothness. Increasing that weight
(e.g., to 1.0) would produce a visibly gentler, slower voltage profile at
the cost of tracking speed -- worth trying if you want to see the tradeoff
directly.

How to reuse this pattern for any new plant/controller you tune later:

    1. Write the plant ODE from first physics (Newton's law / Kirchhoff's law).
    2. Simplify with a stated, justified assumption (we neglected inductance).
    3. Convert to state-space, verify A/B/C by hand against the code's printed output.
    4. For classical control: check System Type first -- it tells you whether
       you even need an integral term before you add one blindly.
    5. For MPC: write out the cost function with your actual N/Q/R values
       before touching code, so you know what behavior you're asking the
       solver to produce.

---

## 7. Nominal vs. Robust MPC (Why Constraints Can Still Be Violated)

**The gap our stress test exposed:** MPC's constraint guarantee
(x_min <= x_k <= x_max, enforced by the solver) is only as good as the
model f(x,u) used inside the optimization. If the true plant experiences
a disturbance the model doesn't know about:

    x_true_{k+1} = f(x_k, u_k) + d_k     (d_k = unmodeled disturbance)

...then the solver's predicted x_k+1 (computed using only f, not d) can
diverge from reality. The constraint was satisfied in the *solver's plan*,
not necessarily in the *real trajectory* that actually unfolds.

**Measured on our plant:** ideal conditions (Task 6) -> constraint held
exactly (2.883 vs 3.0 rad/s limit). Same constraint, same controller,
under an unmodeled disturbance (Task 8) -> violated by 30.4%.

**What actually fixes this (named as future work, not implemented here):**
robust MPC formulations bound the disturbance explicitly, e.g. tube MPC
tightens the constraint by a safety margin computed from the disturbance
bound:

    x_min + margin(d) <= x_k <= x_max - margin(d)

so that even in the worst case within the assumed disturbance bound, the
TRUE state stays within the original limit. This is a real, standard
extension - worth naming explicitly if asked "how would you make this
production-safe," since claiming nominal MPC alone is safety-guaranteed
under real disturbances would be an overclaim.

**Interview-ready one-liner:** "My MPC's constraints held exactly under
ideal conditions, but I explicitly stress-tested it with an unmodeled
disturbance and found the guarantee degrades - about 30% over the limit
versus PD's 49% over. That's expected for nominal MPC, and it's exactly
why robust or tube MPC exists as the production-grade extension."

---

## 8. Damping Ratio From Given Poles (Reverse Direction)

Section 4 went ζ,ωₙ -> poles (design direction). The reverse -- given
measured poles, recover ζ and ωₙ -- is exactly what was used to diagnose
the failed Ki=40 PID attempt in Section 3, but the arithmetic was never
shown. For a complex pole pair p = -a +/- jb:

    omega_n = sqrt(a^2 + b^2)          (distance from origin in s-plane)
    zeta    = a / omega_n              (fraction of that distance along
                                         the negative real axis)

**Worked example (the failed Ki=40 attempt):** measured poles were
-0.8819 +/- j1.9482.

    omega_n = sqrt(0.8819^2 + 1.9482^2) = sqrt(0.7777 + 3.7955) = sqrt(4.5732) = 2.1385
    zeta    = 0.8819 / 2.1385 = 0.4124

Matches the "zeta≈0.41" figure quoted when diagnosing that result --
this is the actual arithmetic behind that number. Rule of thumb: zeta < 0.7
means visibly underdamped/oscillatory; zeta >= 1.0 means no oscillation at
all (overdamped or critically damped).

## 9. Metrics & Definitions Used Throughout

These formulas were implemented directly in code (never derived from
control theory) but are used as the reported numbers in every DEVLOG
entry and benchmark table -- worth having explicit for "how exactly did
you compute X" questions.

**RMS (root-mean-square) tracking error**, over N samples of error e_i:

    RMS = sqrt( (1/N) * sum(e_i^2) )

Used in Task 3's feedforward comparison (87.4% RMS reduction). RMS is
preferred over simple average error because squaring penalizes large
transient errors more than an average would -- a controller with one huge
spike and otherwise-zero error scores worse on RMS than on mean error,
which better reflects real tracking quality.

**Max (peak) tracking error:**

    max_error = max(|e_i|)  for i in 1..N

The single worst instantaneous error -- useful alongside RMS since a
controller can have excellent RMS error but one dangerous spike that RMS
alone would hide.

**Control chatter (Task 8's noise-sensitivity metric):**

    chatter = std( u_{k+1} - u_k )   for all consecutive control samples

The standard deviation of the *change* in control signal between
consecutive steps -- not the standard deviation of u itself. This
specifically measures how much the control signal jumps around step to
step, which is what actually stresses real actuator hardware (a
consistently high but smooth voltage is fine; a rapidly oscillating one
isn't).

**Settling time (2% band) -- exact algorithm used in every script:**

    tolerance = 0.02 * target
    settling_time = first t_i such that |theta_j - target| < tolerance
                    for ALL j >= i   (not just at t_i itself)

This "for all subsequent samples" condition matters: checking only the
instant it first crosses the tolerance band would falsely report settling
if the signal dips back out again later (e.g., during oscillation). This
is why Task 7's N=5 case correctly returned "None" -- theta kept
overshooting far outside the band even after briefly crossing near target.

**Overshoot (percentage above final value):**

    overshoot_pct = (max(theta) - final_value) / final_value * 100    [if max > final]
    overshoot_pct = 0                                                  [otherwise]

Caution (the Task 5 lesson): this formula divides by final_value, so if
final_value is near zero, small numerator differences produce huge,
meaningless percentages (Task 7's N=5 case: -5137.98%). Always sanity-check
overshoot% against the raw theta trace when final_value is small.

## 10. Discretization Scheme (Simulation Methodology)

Every custom simulation loop (PD, feedforward, stress test -- NOT the
do-mpc simulator, which uses its own internal integrator) advances the
continuous-time state-space model using explicit (forward) Euler
integration:

    x_{k+1} = x_k + (A*x_k + B*u_k) * dt

This is the simplest possible numerical integration scheme: assume the
derivative x_dot = Ax+Bu is constant over the small interval dt, and step
forward linearly. It has known accuracy limits (local error scales with
dt^2, global error with dt) -- justified here because dt=0.001s for the
1kHz PD loop and dt=0.02s for the 50Hz MPC loop are both small relative to
the plant's own time constant (~0.12s from Stage 1's poles), so the
approximation error stays small. Worth naming explicitly in the paper's
Methodology section as a stated, deliberate simplification, same
convention as neglecting armature inductance in Section 1.
