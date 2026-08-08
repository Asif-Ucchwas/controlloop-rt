# ISO 26262 Functional Safety Mapping

**Scope and disclaimer:** this document maps ControlLoop-RT's implemented
mechanisms (Tasks 13-15) to ISO 26262 concepts and vocabulary, for
documentation purposes. It is a conceptual/illustrative
mapping, NOT a certified safety case. Real ISO 26262 compliance requires
an independent safety assessment, a qualified toolchain, MISRA C static
analysis, formal requirements traceability (e.g. DOORS), and a dedicated
safety review process - none of which this solo project provides. This
document demonstrates understanding of the standard's structure and
reasoning, not compliance with it.

---

## 1. Item Definition

**Item:** a closed-loop DC servo position-control system, comprising:
- Plant: DC servo motor (Stage 1 model, docs/notes/controls_math_reference.md Section 1)
- Sensing: dual redundant position sensors with voting (Task 14)
- Control: PD/feedforward/MPC controllers (Stages 1-2), running as a
  1kHz periodic RTOS task (Stage 3)
- Monitoring: independent watchdog thread (Task 13)
- Safety response: active-hold safe-state transition (Task 15)

**Operating environment (illustrative):** a general-purpose precision
positioning application - e.g. a robotic joint or actuator requiring
accurate, bounded position control with defined behavior under fault.

---

## 2. Hazard Analysis (HARA) - Lite

Illustrative severity (S0-S3), exposure (E0-E4), controllability (C0-C3)
per ISO 26262-3 conventions. Ratings here are reasoned examples for a
generic precision-positioning application, not derived from a real
target system - a genuine HARA requires a specific vehicle/product
context this project does not have.

### Hazard 1: Uncontrolled motion after a control-loop fault

Description: the control task hangs, crashes, or stops updating, and
the actuator continues moving under its last-commanded input (or coasts
uncontrolled) with no correction.

    Severity (S2):     moderate injury/damage plausible depending on application
    Exposure (E3):     control-loop faults, while rare per-cycle, are a
                        realistic long-run occurrence over a system's lifetime
    Controllability (C2): an operator or downstream system has limited
                        ability to intervene once the fault occurs
    Illustrative ASIL: ASIL B

**Mitigated by:** Task 13's watchdog (detects the fault) + Task 15's
active-hold safe-state (bounds the physical consequence - see Hazard 2).

### Hazard 2: Excessive uncontrolled coast distance after fault

Description: even once a fault is detected, the safety response itself
lets the actuator travel further than acceptable before stopping.

    Severity (S2):     moderate, depends on coast distance vs. clearance
                        to nearest hazard (e.g. obstacle, human)
    Exposure (E3):     same as Hazard 1 - occurs whenever Hazard 1 occurs
    Controllability (C1): the safety mechanism itself is the control;
                        no external intervention expected or needed
    Illustrative ASIL: ASIL B

**Mitigated by:** Task 15's active-hold - measured coast distance
reduced from Task 13's naive-cutoff result (0.48 rad, uncontrolled final
position) to 0.0000 rad final coast distance (bounded transient
overshoot ~0.16 rad, then corrected return to the fault-time position).
This is the single most safety-relevant quantified result in the
project - see DEVLOG Stage 4 Task 15.

### Hazard 3: Erroneous position feedback from a faulty sensor

Description: a single sensor fails (stuck, biased, disconnected) and
the control loop acts on the corrupted reading as if it were true,
potentially driving the actuator to an incorrect and unsafe position.

    Severity (S2):     moderate, actuator could move to a genuinely
                        wrong position based on false feedback
    Exposure (E2):     sensor faults are less frequent than generic
                        control-loop faults but not negligible over
                        a system's operating life
    Controllability (C2): limited external ability to detect a subtly
                        wrong position before consequences occur
    Illustrative ASIL: ASIL B

**Mitigated by:** Task 14's dual-sensor voting - measured zero missed
detections and zero false positives across 200 injected-fault cycles;
quantified that undefended (naive-averaging) operation would have fed a
persistent ~0.25 rad false reading into the control law, which voting
fully prevented from ever reaching the plant (true_theta stayed flat
throughout the fault window).

---

## 3. Safety Goals (derived from the hazards above)

    SG-1: The system shall detect control-loop faults (hangs, missed
          cycles) within a bounded time and transition to a defined
          safe state. [addresses Hazard 1]

    SG-2: The safe-state transition shall minimize uncontrolled motion
          after fault detection, bounding both transient overshoot and
          final resting position relative to the fault-detection point.
          [addresses Hazard 2]

    SG-3: The system shall detect disagreement between redundant
          position sensors and prevent a single faulty sensor from being
          acted upon as ground truth. [addresses Hazard 3]

---

## 4. Functional Safety Concept - Mechanism Traceability

| Safety Goal | Mechanism | Task/Code | Verification Evidence |
|---|---|---|---|
| SG-1 | Independent watchdog thread, priority 3 (higher than control_task), 2500us timeout, checked every 2ms | Task 13, stage4_safety/control_loop_rt/src/main.c (watchdog_task) | Fault injection (withheld kicks) detected in ~3.5-4ms, matching the theoretical bound (timeout + polling period). Zero missed detections across all test runs. |
| SG-2 | Active-hold: re-target existing PD controller to fault-time position instead of cutting power | Task 15, control_task's fault-branch error computation | Direct A/B comparison: naive cutoff = 0.48 rad uncontrolled final coast; active-hold = 0.0000 rad final coast distance (bounded ~0.16 rad transient, then corrected). |
| SG-3 | Dual-sensor voting: agree within 0.05 rad -> average; disagree -> fall back to primary sensor, flag disagreement | Task 14, control_task's sensor-voting block | 200/200 injected-fault cycles correctly detected. Zero false positives in 2800 normal-noise cycles. true_theta stayed undisturbed throughout the fault window - the plant never reacted to the corrupted sensor. |

**Standard ISO 26262 vocabulary these mechanisms correspond to:**
- Watchdog = "program flow monitoring" / independent fault-detection mechanism (ISO 26262-5)
- Dual-sensor voting = a "1002" (1-out-of-2 with comparison) redundancy
  architecture - a standard pattern for detecting (not necessarily
  tolerating) a single sensor fault
- Active-hold = "transition to a safe state" (ISO 26262-4) - specifically
  a degraded-but-controlled operating state, not a full system shutdown

---

## 5. Honest Gap List - What Real Certification Would Require

This project demonstrates the CONCEPTS and produces genuine, measured
evidence for each mechanism - but is not a certified safety case. A real
ISO 26262 program for this system would additionally require:

- Independent safety assessment by a qualified assessor, not self-review
- MISRA C compliance and static analysis (this code was not checked
  against MISRA C rules)
- A qualified/certified toolchain (Zephyr + gcc + native_sim is a
  development/prototyping stack, not a certified embedded toolchain)
- Formal requirements traceability tooling (e.g. DOORS), not a markdown
  table
- Quantitative failure-rate analysis (FMEA/FMEDA) with real component
  datasheet data - this project used illustrative S/E/C ratings, not
  data-driven ones
- Verification on real hardware, not exclusively native_sim (see Stage
  7, optional/deferred) - native_sim's idealized timing model (Stage 3
  Task 12) cannot itself serve as timing certification evidence
- A dedicated, resourced safety review process, not a solo repository

**Summary:** "I built and tested real fault-detection and
safe-state mechanisms with quantified evidence for each, and I can map
that work to ISO 26262's structure and vocabulary - but I'm not claiming
certification. I know specifically what a real safety program adds on
top of this, and why each piece matters."
