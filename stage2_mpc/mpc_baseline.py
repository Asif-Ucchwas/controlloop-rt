import matplotlib
matplotlib.use('Agg')
import numpy as np
import do_mpc
import matplotlib.pyplot as plt

# Same simplified 2nd-order DC motor model as Stage 1.
# Full derivation: docs/notes/controls_math_reference.md, Section 1
J, b, K, R = 0.01, 0.1, 0.01, 1.0

# --- Build the do-mpc model ---
model = do_mpc.model.Model('continuous')
theta = model.set_variable('_x', 'theta')
theta_dot = model.set_variable('_x', 'theta_dot')
V = model.set_variable('_u', 'V')

model.set_rhs('theta', theta_dot)
model.set_rhs('theta_dot', -(b + K**2/R)/J * theta_dot + K/(R*J) * V)
model.setup()

# --- Build the MPC controller ---
mpc = do_mpc.controller.MPC(model)
mpc.set_param(n_horizon=20, t_step=0.02, n_robust=0, store_full_solution=True)

# Silence IPOPT's per-iteration solver logging — otherwise this floods the terminal
mpc.set_param(nlpsol_opts={'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0})

target = 1.0
mterm = (model.x['theta'] - target)**2                # terminal cost
lterm = (model.x['theta'] - target)**2                # running cost
mpc.set_objective(mterm=mterm, lterm=lterm)
mpc.set_rterm(V=0.01)   # small penalty on control effort/chatter

# Voltage bounds — generous for now, Task 6 tightens these deliberately
mpc.bounds['lower', '_u', 'V'] = -100
mpc.bounds['upper', '_u', 'V'] = 100

mpc.setup()

# --- Simulator: advances the "real" plant given MPC's chosen input ---
simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step=0.02)
simulator.setup()

estimator = do_mpc.estimator.StateFeedback(model)

x0 = np.array([[0.0], [0.0]])
mpc.x0 = x0
simulator.x0 = x0
mpc.set_initial_guess()

# --- Closed-loop simulation ---
T_end = 3.0
steps = int(T_end / 0.02)
t_log, theta_log, u_log = [], [], []

for i in range(steps):
    u0 = mpc.make_step(x0)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)

    t_log.append(i * 0.02)
    theta_log.append(x0[0, 0])
    u_log.append(u0[0, 0])

t_log, theta_log, u_log = np.array(t_log), np.array(theta_log), np.array(u_log)

# --- Same metrics as Task 2, for direct comparison ---
final_value = theta_log[-1]
overshoot = (max(theta_log) - final_value) / final_value * 100 if max(theta_log) > final_value else 0.0
tolerance = 0.02 * target
settled_idx = None
for i in range(len(theta_log)):
    if np.all(np.abs(theta_log[i:] - target) < tolerance):
        settled_idx = i
        break
settling_time = t_log[settled_idx] if settled_idx is not None else None
steady_state_error = abs(target - final_value)

print(f"Target angle: {target} rad")
print(f"Final value: {final_value:.4f} rad")
print(f"Overshoot: {overshoot:.2f}%")
print(f"Settling time (2% band): {settling_time} s")
print(f"Steady-state error: {steady_state_error:.5f} rad")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(t_log, theta_log, label="θ (MPC)")
ax1.axhline(target, color='r', linestyle='--', label="target")
ax1.set_ylabel("Shaft angle (rad)")
ax1.set_title("MPC — Step Response")
ax1.legend(); ax1.grid(True)

ax2.plot(t_log, u_log, color='green')
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Control effort (V)")
ax2.set_title("MPC Control Input")
ax2.grid(True)

plt.tight_layout()
plt.savefig("stage2_mpc/mpc_step_response.png", dpi=120)
print("\nSaved plot: stage2_mpc/mpc_step_response.png")
