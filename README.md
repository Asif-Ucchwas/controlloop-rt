# ControlLoop-RT

**Model-Predictive Control, Feedforward & Real-Time Safety-Critical Systems**

Md Asifuzzaman

A from-scratch controls engineering system spanning classical feedback
control, feedforward, constrained MPC, real-time RTOS integration, and
functional-safety mechanisms — all validated on the same simulated DC
servo plant, with every claim backed by measured data, not assertion.

---

## What This Project Demonstrates

    - Classical control:      PID -> diagnosed failure -> PD via pole placement
    - Feedforward control:    87.4% RMS tracking error reduction (moving reference)
    - Model Predictive Control: constrained QP, receding horizon, do-mpc/CasADi
    - Real-time embedded:     ported into a genuine 1kHz Zephyr RTOS task
    - Functional safety:      watchdog, dual-sensor voting, active-hold safe-state
    - Rigorous benchmarking:  600 trials (3 controllers x 4 conditions x 50 trials)
    - ISO 26262 mapping:      documented (not certified) hazard analysis & traceability

**Every stage in this project found and fixed at least one real bug or
design flaw during development, not just at the end** — see DEVLOG.md
for the full, honest debugging history.

---

## Architecture

    Stage 1: Plant Model & Baseline
      DC Servo Motor (2nd-order ODE) --> State-Space --> PD Controller
                                                       --> Feedforward (model inversion)

    Stage 2: Model Predictive Control
      Same Plant --> do-mpc/CasADi QP Solver (N=20, 50Hz)
                  --> Hard constraints (actuator, velocity)
                  --> Stress-tested under sensor noise + disturbance

    Stage 3: Real-Time Integration (Zephyr RTOS, native_sim)
      PD Control Loop --> k_timer (1kHz) --> k_sem --> Periodic Task
                       --> Competing lower-priority task --> Preemption proven
                       --> Cycle-resolution jitter measurement (zero jitter, stress-tested)

    Stage 4: Functional Safety
      Watchdog Thread (priority 3) --> monitors --> Control Task (priority 5)
      Dual Sensor Channels --> Voting Logic --> Control Task (never reads raw sensors)
      Fault Detected --> Active-Hold Safe-State (not power cutoff)
      --> Mapped to ISO 26262 (HARA, safety goals, traceability)

    Stage 5: Benchmarking
      PID / PD+Feedforward / MPC --> 4 conditions x 50 trials each
                                  --> RMS/max error, chatter, settling time, constraint violations
                                  --> 600 total simulation runs, statistically aggregated

---

## Repository Structure

    stage1_plant/           Plant model, PD baseline, feedforward (Python)
    stage2_mpc/              MPC formulation, constraints, horizon tuning, stress tests (Python)
    stage3_rtos/             Real-time Zephyr port: periodic task, preemption, jitter (C)
    stage4_safety/           Watchdog, sensor voting, active-hold safe-state (C)
    stage5_benchmarking/     Test protocol, 600-trial runner, plots, results table (Python)
    docs/notes/               Math reference doc, ISO 26262 mapping, plant derivation
    assets/                   Benchmark comparison plots (README-embedded)
    DEVLOG.md                 Full running development log — every bug, every fix, honestly
    requirements.txt           Python dependencies (Stages 1, 2, 5)

---

## Setup

### Python environment (Stages 1, 2, 5)

    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt

Run any stage's scripts directly, e.g.:

    python3 stage1_plant/dc_servo_model.py
    python3 stage5_benchmarking/run_full_suite.py   # full 600-trial suite, ~16 min

### Zephyr RTOS environment (Stages 3, 4)

Requires a working Zephyr SDK + west workspace (this project reused an
existing CAN-Net Zephyr workspace rather than duplicating a ~1GB+
download — see DEVLOG.md Stage 3 Task 9 for the exact west build -s/-d
pattern for building an out-of-tree app against a shared workspace).

    west build -s stage3_rtos/control_loop_rt -d stage3_rtos/build -b native_sim --pristine
    ./stage3_rtos/build/zephyr/zephyr.exe

---

## Key Results

### Feedforward (Stage 1)
87.4% RMS tracking error reduction over feedback-only, on a moving
sinusoidal reference. No benefit on step inputs (the plant's built-in
integrator already zeroes step error) — feedforward's value is
trajectory-tracking specific, and this is stated explicitly rather than
overclaimed.

### MPC vs. Classical Control (Stage 2)
MPC does NOT win on raw unconstrained tracking speed — PD settles
faster (0.447s vs 0.94s) on an unconstrained step. MPC's real advantage:
explicit state-constraint enforcement (PD violated a 3.0 rad/s velocity
limit by 81%; MPC respected it by construction) and dramatically lower
control-effort chatter under sensor noise (14x lower than PD).

### Real-Time Validation (Stage 3)
Ported into an actual 1kHz Zephyr task. Found and fixed a derivative-
kick bug and a silent tick-rate mismatch (requested 1ms, native_sim
silently rounded to 10ms). Stress-tested jitter at 1us resolution under
adversarial competing load: genuinely zero jitter — a property of
native_sim's idealized simulation model, proven by actively trying to
break it, not assumed.

### Functional Safety (Stage 4)
Naive power-cutoff on fault detection let the shaft coast 0.48 rad
uncontrolled. Redesigned as active-hold (re-target the existing PD
controller to the fault-time position): reduced final coast distance to
0.0000 rad (with an honest, physically unavoidable ~0.16 rad transient
overshoot en route). Dual-sensor voting caught 200/200 injected sensor
faults with zero false positives, fully protecting the plant from any
physical disturbance during the fault.

### Benchmark Results (Stage 5, 600 trials)
No controller wins outright — the honest, multi-dimensional finding:
- **Feedforward**: best baseline tracking, but by far the most fragile
  under disturbance (+236% max error vs. PID's +35%, MPC's +13%)
- **MPC**: best average tracking, most robust to disturbance shocks, but
  structurally slower to settle (50Hz vs. 1kHz loop rate) and less
  trial-to-trial consistent under combined stress
- **PID**: mediocre but the most predictable and stable across conditions

Full results: stage5_benchmarking/RESULTS_TABLE.md and assets/*.png

---

## Documentation

    docs/notes/controls_math_reference.md       Every formula used, derived, with worked
                                                   numeric examples using this project's real
                                                   numbers
    docs/notes/plant_model_and_baseline.md       Stage 1 plant derivation & baseline results
    docs/notes/iso26262_functional_safety_mapping.md
                                                   Documented (not certified) ISO 26262 mapping:
                                                   HARA, safety goals, full traceability table
    DEVLOG.md                                     Complete development history — every bug found,
                                                   diagnosed, and fixed, with root causes

---

## Path to Production

This is a portfolio/learning project, not a production or certified
safety-critical system. What would genuinely change to get there:

    - Simulink/MATLAB or a certified modeling environment in place of
      Python (do-mpc/CasADi is excellent for prototyping, not certified
      for production safety-critical deployment)
    - VxWorks, QNX, or a certified AUTOSAR RTOS in place of Zephyr's
      native_sim (a real-time OS proven on real hardware, not a
      software simulation of one)
    - Formal IEC 61508/ISO 26262 certification, including independent
      safety assessment — not the conceptual/illustrative mapping in
      docs/notes/iso26262_functional_safety_mapping.md
    - MISRA C compliance and static analysis on all embedded C code
      (not checked against MISRA C rules in this project)
    - Requirements-traceability tooling (e.g. DOORS), not a markdown table
    - A dedicated, resourced safety review process, not a solo repository
    - Real hardware validation — native_sim's simulated clock is
      deterministic and structurally excludes physical jitter sources
      (Stage 3 Task 12); genuine timing/jitter numbers require an actual
      microcontroller and actuator

See docs/notes/iso26262_functional_safety_mapping.md for the full,
itemized gap list.

---

**GitHub:** github.com/Asif-Ucchwas/controlloop-rt
