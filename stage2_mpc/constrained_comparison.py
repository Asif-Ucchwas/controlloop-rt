import matplotlib
matplotlib.use('Agg')
import numpy as np
import do_mpc
import matplotlib.pyplot as plt

J, b, K, R = 0.01, 0.1, 0.01, 1.0
V_LIMIT = 5.0   # realistic supply rail — now genuinely binding

# --- PD baseline WITH naive post-hoc voltage clamping ---
Kp, Kd = 144.0, 6.79
dt = 0.001
T_end = 3.0
steps_pd = int(T_end / dt)
target = 1.0

x = np.array([[0.0], [0.0]])
prev_error = 0.0
t_pd, theta_pd, u_pd = [], [], []
for i in range(steps_pd):
    t = i * dt
    theta = x[0, 0]
    error = target - theta
    derivative = (error - prev_error) / dt
    u_ideal = Kp*error + Kd*derivative
    u = np.clip(u_ideal, -V_LIMIT, V_LIMIT)   # naive clamp — the "PID doesn't know" problem
    prev_error = error

    A = np.array([[0,1],[0,-(b+K**2/R)/J]])
    B = np.array([[0],[K/(R*J)]])
    x = x + (A @ x + B*u) * dt

    t_pd.append(t); theta_pd.append(theta); u_pd.append(u)

t_pd, theta_pd, u_pd = np.array(t_pd), np.array(theta_pd), np.array(u_pd)

# --- MPC WITH the same constraint built into the optimization ---
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
mpc.bounds['lower', '_u', 'V'] = -V_LIMIT
mpc.bounds['upper', '_u', 'V'] = V_LIMIT
mpc.setup()

simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step=0.02)
simulator.setup()
estimator = do_mpc.estimator.StateFeedback(model)

x0 = np.array([[0.0], [0.0]])
mpc.x0 = x0
simulator.x0 = x0
mpc.set_initial_guess()

steps_mpc = int(T_end / 0.02)
t_mpc, theta_mpc, u_mpc = [], [], []
for i in range(steps_mpc):
    u0 = mpc.make_step(x0)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)
    t_mpc.append(i*0.02); theta_mpc.append(x0[0,0]); u_mpc.append(u0[0,0])

t_mpc, theta_mpc, u_mpc = np.array(t_mpc), np.array(theta_mpc), np.array(u_mpc)

def metrics(t, theta, target):
    final_value = theta[-1]
    overshoot = (max(theta) - final_value) / final_value * 100 if max(theta) > final_value else 0.0
    tolerance = 0.02 * target
    settled_idx = None
    for i in range(len(theta)):
        if np.all(np.abs(theta[i:] - target) < tolerance):
            settled_idx = i
            break
    settling_time = t[settled_idx] if settled_idx is not None else None
    return final_value, overshoot, settling_time

fv_pd, os_pd, st_pd = metrics(t_pd, theta_pd, target)
fv_mpc, os_mpc, st_mpc = metrics(t_mpc, theta_mpc, target)

print(f"--- Under {V_LIMIT}V saturation ---")
print(f"PD (naive clamp):  final={fv_pd:.4f}  overshoot={os_pd:.2f}%  settling={st_pd}")
print(f"MPC (constrained): final={fv_mpc:.4f}  overshoot={os_mpc:.2f}%  settling={st_mpc}")
print(f"Max |V| commanded — PD: {np.max(np.abs(u_pd)):.2f}V | MPC: {np.max(np.abs(u_mpc)):.2f}V")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8))
ax1.plot(t_pd, theta_pd, label='PD (clamped)')
ax1.plot(t_mpc, theta_mpc, label='MPC (constrained)')
ax1.axhline(target, color='r', linestyle='--', label='target')
ax1.set_ylabel("θ (rad)"); ax1.set_title(f"Step Response Under {V_LIMIT}V Saturation")
ax1.legend(); ax1.grid(True)

ax2.plot(t_pd, u_pd, label='PD voltage (clamped)')
ax2.plot(t_mpc, u_mpc, label='MPC voltage')
ax2.axhline(V_LIMIT, color='gray', linestyle=':'); ax2.axhline(-V_LIMIT, color='gray', linestyle=':')
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Voltage (V)")
ax2.legend(); ax2.grid(True)

plt.tight_layout()
plt.savefig("stage2_mpc/constrained_comparison.png", dpi=120)
print("\nSaved plot: stage2_mpc/constrained_comparison.png")
