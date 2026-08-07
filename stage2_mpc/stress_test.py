import matplotlib
matplotlib.use('Agg')
import numpy as np
import do_mpc
import matplotlib.pyplot as plt

J, b, K, R = 0.01, 0.1, 0.01, 1.0
target = 1.0
V_MAX = 100.0
OMEGA_MAX = 3.0
N_HORIZON = 20   # Task 7's chosen configuration

np.random.seed(42)   # same scenario for both controllers, reproducible

# Disturbance: extra torque-equivalent term active during t in [1.0, 1.3]s
def disturbance(t):
    return -80.0 if 1.0 <= t < 1.3 else 0.0   # rad/s^2 equivalent added to theta_ddot

THETA_NOISE_STD = 0.01      # rad — realistic small encoder noise
OMEGA_NOISE_STD = 0.05      # rad/s — velocity estimate is noisier, typically derived/differentiated

def true_dynamics_step(x_true, u, dt, t):
    A = np.array([[0,1],[0,-(b+K**2/R)/J]])
    B = np.array([[0],[K/(R*J)]])
    dist = np.array([[0],[disturbance(t)]])
    x_dot = A @ x_true + B*u + dist
    return x_true + x_dot*dt

def measure(x_true):
    noisy = x_true.copy()
    noisy[0,0] += np.random.normal(0, THETA_NOISE_STD)
    noisy[1,0] += np.random.normal(0, OMEGA_NOISE_STD)
    return noisy

# --- PD controller under noise + disturbance ---
Kp, Kd = 144.0, 6.79
dt = 0.001
steps_pd = int(3.0/dt)
x_true = np.array([[0.0],[0.0]])
prev_error = 0.0
t_pd, theta_true_pd, omega_true_pd, u_pd = [], [], [], []
for i in range(steps_pd):
    t = i*dt
    x_meas = measure(x_true)
    error = target - x_meas[0,0]
    derivative = (error - prev_error)/dt
    u = np.clip(Kp*error + Kd*derivative, -V_MAX, V_MAX)
    prev_error = error
    x_true = true_dynamics_step(x_true, u, dt, t)
    t_pd.append(t); theta_true_pd.append(x_true[0,0]); omega_true_pd.append(x_true[1,0]); u_pd.append(u)
t_pd, theta_true_pd, omega_true_pd, u_pd = map(np.array, [t_pd, theta_true_pd, omega_true_pd, u_pd])

# --- MPC under noise + disturbance (model has NO knowledge of the disturbance) ---
model = do_mpc.model.Model('continuous')
theta_v = model.set_variable('_x', 'theta')
theta_dot_v = model.set_variable('_x', 'theta_dot')
V = model.set_variable('_u', 'V')
model.set_rhs('theta', theta_dot_v)
model.set_rhs('theta_dot', -(b + K**2/R)/J * theta_dot_v + K/(R*J) * V)
model.setup()

mpc = do_mpc.controller.MPC(model)
mpc.set_param(n_horizon=N_HORIZON, t_step=0.02, n_robust=0, store_full_solution=True)
mpc.set_param(nlpsol_opts={'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0})
mterm = (model.x['theta'] - target)**2
lterm = (model.x['theta'] - target)**2
mpc.set_objective(mterm=mterm, lterm=lterm)
mpc.set_rterm(V=0.01)
mpc.bounds['lower', '_u', 'V'] = -V_MAX
mpc.bounds['upper', '_u', 'V'] = V_MAX
mpc.bounds['lower', '_x', 'theta_dot'] = -OMEGA_MAX
mpc.bounds['upper', '_x', 'theta_dot'] = OMEGA_MAX
mpc.setup()

x_true = np.array([[0.0],[0.0]])
mpc.x0 = x_true
mpc.set_initial_guess()

dt_mpc = 0.02
steps_mpc = int(3.0/dt_mpc)
t_mpc, theta_true_mpc, omega_true_mpc, u_mpc = [], [], [], []
for i in range(steps_mpc):
    t = i*dt_mpc
    x_meas = measure(x_true)
    u0 = mpc.make_step(x_meas)
    x_true = true_dynamics_step(x_true, u0[0,0], dt_mpc, t)
    t_mpc.append(t); theta_true_mpc.append(x_true[0,0]); omega_true_mpc.append(x_true[1,0]); u_mpc.append(u0[0,0])
t_mpc, theta_true_mpc, omega_true_mpc, u_mpc = map(np.array, [t_mpc, theta_true_mpc, omega_true_mpc, u_mpc])

# --- Metrics ---
def analyze(t, theta, omega, u, label):
    final_value = theta[-1]
    tolerance = 0.02*target
    settled_idx = None
    for i in range(len(theta)):
        if np.all(np.abs(theta[i:] - target) < tolerance):
            settled_idx = i
            break
    settling = t[settled_idx] if settled_idx is not None else None
    peak_omega = np.max(np.abs(omega))
    violated = peak_omega > OMEGA_MAX
    violation_margin = (peak_omega - OMEGA_MAX)/OMEGA_MAX*100 if violated else 0.0
    steady_error = abs(target - final_value)
    control_chatter = np.std(np.diff(u))   # noise sensitivity proxy
    print(f"{label}: settling={settling}  peak|omega|={peak_omega:.3f}  "
          f"constraint_violated={violated} (margin={violation_margin:.1f}%)  "
          f"final_error={steady_error:.4f}  control_chatter_std={control_chatter:.3f}")
    return violated, violation_margin

print("--- Stress Test: Sensor Noise + Unmodeled Disturbance (t=1.0-1.3s) ---")
analyze(t_pd, theta_true_pd, omega_true_pd, u_pd, "PD ")
analyze(t_mpc, theta_true_mpc, omega_true_mpc, u_mpc, "MPC")

fig, axes = plt.subplots(3, 1, figsize=(9, 10))
axes[0].plot(t_pd, theta_true_pd, label='PD', alpha=0.8)
axes[0].plot(t_mpc, theta_true_mpc, label='MPC', alpha=0.8)
axes[0].axhline(target, color='r', linestyle='--')
axes[0].axvspan(1.0, 1.3, color='gray', alpha=0.2, label='disturbance window')
axes[0].set_ylabel("θ (rad)"); axes[0].legend(); axes[0].grid(True)
axes[0].set_title("Position Under Noise + Disturbance")

axes[1].plot(t_pd, omega_true_pd, label='PD θ̇')
axes[1].plot(t_mpc, omega_true_mpc, label='MPC θ̇')
axes[1].axhline(OMEGA_MAX, color='gray', linestyle=':')
axes[1].axhline(-OMEGA_MAX, color='gray', linestyle=':')
axes[1].axvspan(1.0, 1.3, color='gray', alpha=0.2)
axes[1].set_ylabel("θ̇ (rad/s)"); axes[1].legend(); axes[1].grid(True)

axes[2].plot(t_pd, u_pd, label='PD voltage', alpha=0.7)
axes[2].plot(t_mpc, u_mpc, label='MPC voltage', alpha=0.7)
axes[2].axvspan(1.0, 1.3, color='gray', alpha=0.2)
axes[2].set_xlabel("Time (s)"); axes[2].set_ylabel("Voltage (V)")
axes[2].legend(); axes[2].grid(True)

plt.tight_layout()
plt.savefig("stage2_mpc/stress_test.png", dpi=120)
print("\nSaved plot: stage2_mpc/stress_test.png")
