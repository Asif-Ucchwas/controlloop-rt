import matplotlib
matplotlib.use('Agg')
import numpy as np
import do_mpc
import matplotlib.pyplot as plt

J, b, K, R = 0.01, 0.1, 0.01, 1.0
target = 1.0
V_MAX = 100.0        # generous voltage limit — not the binding constraint this time
OMEGA_MAX = 3.0       # rad/s — the state limit we actually care about

# --- PD baseline: structurally CANNOT enforce a velocity limit ---
Kp, Kd = 144.0, 6.79
dt = 0.001
steps_pd = int(3.0 / dt)
x = np.array([[0.0], [0.0]])
prev_error = 0.0
t_pd, theta_pd, omega_pd = [], [], []
for i in range(steps_pd):
    t = i * dt
    theta, omega = x[0,0], x[1,0]
    error = target - theta
    derivative = (error - prev_error) / dt
    u = np.clip(Kp*error + Kd*derivative, -V_MAX, V_MAX)
    prev_error = error
    A = np.array([[0,1],[0,-(b+K**2/R)/J]])
    B = np.array([[0],[K/(R*J)]])
    x = x + (A @ x + B*u) * dt
    t_pd.append(t); theta_pd.append(theta); omega_pd.append(omega)
t_pd, theta_pd, omega_pd = np.array(t_pd), np.array(theta_pd), np.array(omega_pd)

# --- MPC: velocity limit added as an explicit state constraint ---
model = do_mpc.model.Model('continuous')
theta_v = model.set_variable('_x', 'theta')
theta_dot_v = model.set_variable('_x', 'theta_dot')
V = model.set_variable('_u', 'V')
model.set_rhs('theta', theta_dot_v)
model.set_rhs('theta_dot', -(b + K**2/R)/J * theta_dot_v + K/(R*J) * V)
model.setup()

mpc = do_mpc.controller.MPC(model)
mpc.set_param(n_horizon=20, t_step=0.02, n_robust=0, store_full_solution=True)
mpc.set_param(nlpsol_opts={'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0})
mterm = (model.x['theta'] - target)**2
lterm = (model.x['theta'] - target)**2
mpc.set_objective(mterm=mterm, lterm=lterm)
mpc.set_rterm(V=0.01)
mpc.bounds['lower', '_u', 'V'] = -V_MAX
mpc.bounds['upper', '_u', 'V'] = V_MAX
# THE key line — a state constraint PID has no equivalent mechanism for:
mpc.bounds['lower', '_x', 'theta_dot'] = -OMEGA_MAX
mpc.bounds['upper', '_x', 'theta_dot'] = OMEGA_MAX
mpc.setup()

simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step=0.02)
simulator.setup()
estimator = do_mpc.estimator.StateFeedback(model)
x0 = np.array([[0.0], [0.0]])
mpc.x0 = x0; simulator.x0 = x0
mpc.set_initial_guess()

steps_mpc = int(3.0 / 0.02)
t_mpc, theta_mpc, omega_mpc = [], [], []
for i in range(steps_mpc):
    u0 = mpc.make_step(x0)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)
    t_mpc.append(i*0.02); theta_mpc.append(x0[0,0]); omega_mpc.append(x0[1,0])
t_mpc, theta_mpc, omega_mpc = np.array(t_mpc), np.array(theta_mpc), np.array(omega_mpc)

print(f"Velocity limit: ±{OMEGA_MAX} rad/s")
print(f"PD  peak |theta_dot|:  {np.max(np.abs(omega_pd)):.3f} rad/s  →  {'VIOLATED' if np.max(np.abs(omega_pd)) > OMEGA_MAX else 'within limit'}")
print(f"MPC peak |theta_dot|:  {np.max(np.abs(omega_mpc)):.3f} rad/s  →  {'VIOLATED' if np.max(np.abs(omega_mpc)) > OMEGA_MAX else 'within limit'}")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8))
ax1.plot(t_pd, theta_pd, label='PD')
ax1.plot(t_mpc, theta_mpc, label='MPC')
ax1.axhline(target, color='r', linestyle='--', label='target')
ax1.set_ylabel("θ (rad)"); ax1.legend(); ax1.grid(True)
ax1.set_title("Position — PD vs. MPC with velocity constraint")

ax2.plot(t_pd, omega_pd, label='PD θ̇')
ax2.plot(t_mpc, omega_mpc, label='MPC θ̇')
ax2.axhline(OMEGA_MAX, color='gray', linestyle=':', label='velocity limit')
ax2.axhline(-OMEGA_MAX, color='gray', linestyle=':')
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("θ̇ (rad/s)")
ax2.legend(); ax2.grid(True)

plt.tight_layout()
plt.savefig("stage2_mpc/state_constraint_comparison.png", dpi=120)
print("\nSaved plot: stage2_mpc/state_constraint_comparison.png")
