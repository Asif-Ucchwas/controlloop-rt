# Plant Model & Baseline Control — Stage 1

## Plant: DC Servo Motor

**Physical parameters** (typical small servo motor):
| Parameter | Symbol | Value | Units |
|---|---|---|---|
| Rotor inertia | J | 0.01 | kg·m² |
| Viscous friction | b | 0.1 | N·m·s |
| Motor constant | K | 0.01 | N·m/A |
| Armature resistance | R | 1.0 | Ω |

**Simplifying assumption:** armature inductance neglected (electrical time
constant << mechanical time constant) — standard 2nd-order DC servo approximation.

**State-space model** (x = [θ, θ̇], u = V, y = θ):
**Transfer function:**
Note the built-in integrator (`s` factor in denominator) — this plant naturally
drives step-reference error to zero without an explicit integral controller term.

## Baseline Controller: PD (not PID)

**Design method:** pole placement. Closed-loop characteristic equation with
Ki=0: `s² + (10.01 + Kd)s + Kp = 0`. Matched to `s² + 2ζωₙs + ωₙ² = 0`
with ζ=0.7, ωₙ=12 rad/s.
**Why not PID:** an initial attempt with Ki=40 produced a 3rd-order closed
loop with a lightly-damped oscillatory mode (poles at -0.88 ± j1.95,
ζ≈0.41), giving 54% overshoot and no settling within 3s. Root cause: the
plant already contains an integrator, so adding Ki created a redundant
second integrator rather than eliminating steady-state error. Removed Ki
entirely; PD alone gives zero steady-state error to step inputs because of
the plant's inherent integration.

### Step Response (1 rad target)
| Metric | Value |
|---|---|
| Overshoot | 6.06% |
| Settling time (2% band) | 0.447 s |
| Steady-state error | 0.00000 rad |
| Closed-loop poles | -8.4 ± j8.57 (predicted: -8.4 ± j8.57) |

## Feedforward Addition

**Feedforward law** (model inversion):
Tested on a sinusoidal reference (1 rad amplitude, 0.5 Hz) — a moving
target, since feedforward provides no benefit on a static step (the plant's
built-in integrator already zeroes step error under feedback alone).

### Tracking Comparison — Feedback-only vs. Feedback+Feedforward
| Metric | Feedback-only | Feedback+Feedforward | Improvement |
|---|---|---|---|
| RMS tracking error | 0.16100 rad | 0.02026 rad | 87.4% reduction |
| Max tracking error | 0.23171 rad | 0.12076 rad | 47.9% reduction |

**Note on residual max error:** the feedback+feedforward max error occurs
during the initial transient (t≈0), where the plant starts at rest while
the reference trajectory is already in motion — feedforward compensates for
steady-state dynamics but cannot instantaneously correct an initial-condition
mismatch. This is a known, expected limitation, not a tuning failure.

## Comparison Point for Stage 2

This PD+feedforward baseline (87.4% RMS tracking error reduction over
feedback-only, 0.447s settling time on step response) is the benchmark
Stage 2's MPC controller will be measured against.
