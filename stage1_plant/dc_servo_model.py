import matplotlib
matplotlib.use('Agg')  # headless WSL2 — no display backend
import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Simplified 2nd-order DC motor model — armature inductance (L) neglected,
# since electrical dynamics settle much faster than mechanical ones.
# Full derivation: docs/notes/controls_math_reference.md, Section 1

# --- Motor parameters (typical small DC servo, e.g. Pittman/Maxon-class) ---
J = 0.01      # rotor inertia (kg·m^2)
b = 0.1       # viscous friction coefficient (N·m·s)
K = 0.01      # motor torque/back-EMF constant (N·m/A or V·s/rad)
R = 1.0       # armature resistance (Ohms)

# --- Build state-space model: x = [theta, theta_dot], u = V, y = theta ---
A = np.array([[0, 1],
              [0, -(b + K**2 / R) / J]])
B = np.array([[0],
              [K / (R * J)]])
C = np.array([[1, 0]])
D = np.array([[0]])

sys = ct.ss(A, B, C, D)
print("State-space model:")
print(sys)

# --- Sanity check: step response to a 1V input ---
t = np.linspace(0, 5, 500)
t_out, y_out = ct.step_response(sys, T=t)

plt.figure(figsize=(8, 5))
plt.plot(t_out, y_out)
plt.xlabel("Time (s)")
plt.ylabel("Shaft angle θ (rad)")
plt.title("DC Servo Motor — Open-Loop Step Response (1V input)")
plt.grid(True)
plt.savefig("stage1_plant/open_loop_step_response.png", dpi=120)
print("\nSaved plot: stage1_plant/open_loop_step_response.png")

# --- Report transfer function too, since it's the more common controls language ---
tf_sys = ct.ss2tf(sys)
print("\nEquivalent transfer function θ(s)/V(s):")
print(tf_sys)
