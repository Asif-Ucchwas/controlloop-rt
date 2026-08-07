import matplotlib
matplotlib.use('Agg')
import numpy as np
import do_mpc
import matplotlib.pyplot as plt
import time

J, b, K, R = 0.01, 0.1, 0.01, 1.0
target = 1.0
V_MAX = 100.0
OMEGA_MAX = 3.0

def run_mpc(n_horizon, t_step=0.02, T_end=3.0):
    model = do_mpc.model.Model('continuous')
    theta_v = model.set_variable('_x', 'theta')
    theta_dot_v = model.set_variable('_x', 'theta_dot')
    V = model.set_variable('_u', 'V')
    model.set_rhs('theta', theta_dot_v)
    model.set_rhs('theta_dot', -(b + K**2/R)/J * theta_dot_v + K/(R*J) * V)
    model.setup()

    mpc = do_mpc.controller.MPC(model)
    mpc.set_param(n_horizon=n_horizon, t_step=t_step, n_robust=0, store_full_solution=True)
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

    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step=t_step)
    simulator.setup()
    estimator = do_mpc.estimator.StateFeedback(model)

    x0 = np.array([[0.0], [0.0]])
    mpc.x0 = x0; simulator.x0 = x0
    mpc.set_initial_guess()

    steps = int(T_end / t_step)
    t_log, theta_log, omega_log, solve_times = [], [], [], []
    for i in range(steps):
        t0 = time.perf_counter()
        u0 = mpc.make_step(x0)
        solve_times.append(time.perf_counter() - t0)
        y_next = simulator.make_step(u0)
        x0 = estimator.make_step(y_next)
        t_log.append(i*t_step); theta_log.append(x0[0,0]); omega_log.append(x0[1,0])

    return (np.array(t_log), np.array(theta_log), np.array(omega_log),
            np.array(solve_times))

horizons = [5, 10, 20, 40]
results = {}
for N in horizons:
    print(f"Running N={N}...")
    t, theta, omega, solve_times = run_mpc(N)
    final_value = theta[-1]
    tolerance = 0.02 * target
    settled_idx = None
    for i in range(len(theta)):
        if np.all(np.abs(theta[i:] - target) < tolerance):
            settled_idx = i
            break
    settling_time = t[settled_idx] if settled_idx is not None else None
    overshoot = (max(theta) - final_value)/final_value*100 if max(theta) > final_value else 0.0
    results[N] = {
        'settling': settling_time,
        'overshoot': overshoot,
        'peak_omega': np.max(np.abs(omega)),
        'mean_solve_ms': np.mean(solve_times)*1000,
        'max_solve_ms': np.max(solve_times)*1000,
        't': t, 'theta': theta
    }
    print(f"  settling={settling_time}  overshoot={overshoot:.2f}%  "
          f"peak_omega={np.max(np.abs(omega)):.3f}  "
          f"mean_solve={np.mean(solve_times)*1000:.2f}ms  "
          f"max_solve={np.max(solve_times)*1000:.2f}ms")

print("\n--- Horizon Sweep Summary ---")
print(f"{'N':>4} {'Settling(s)':>12} {'Overshoot%':>12} {'PeakOmega':>10} {'MeanSolve(ms)':>14} {'MaxSolve(ms)':>13}")
for N in horizons:
    r = results[N]
    print(f"{N:>4} {str(r['settling']):>12} {r['overshoot']:>12.2f} {r['peak_omega']:>10.3f} {r['mean_solve_ms']:>14.2f} {r['max_solve_ms']:>13.2f}")

fig, ax = plt.subplots(figsize=(9, 6))
for N in horizons:
    ax.plot(results[N]['t'], results[N]['theta'], label=f'N={N}')
ax.axhline(target, color='r', linestyle='--', label='target')
ax.set_xlabel("Time (s)"); ax.set_ylabel("θ (rad)")
ax.set_title("MPC Step Response — Horizon Sweep")
ax.legend(); ax.grid(True)
plt.tight_layout()
plt.savefig("stage2_mpc/horizon_sweep.png", dpi=120)
print("\nSaved plot: stage2_mpc/horizon_sweep.png")
