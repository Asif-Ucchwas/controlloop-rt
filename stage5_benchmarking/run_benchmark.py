"""
Stage 5 Task 18-19: Benchmark instrumentation and runner.
Protocol: stage5_benchmarking/TEST_PROTOCOL.md
Metric definitions: Section 9 of docs/notes/controls_math_reference.md,
plus the sinusoidal-settling-time redefinition in TEST_PROTOCOL.md.
"""
import numpy as np
import do_mpc

# --- Plant parameters (Stage 1, Section 1 of math reference doc) ---
J, b, K, R = 0.01, 0.1, 0.01, 1.0
A21 = -(b + K**2/R) / J
B21 = K / (R * J)

# --- Controller gains (Stage 1 pole placement, Section 4) ---
KP, KD = 144.0, 6.79

# --- Reference trajectory (fixed across all trials) ---
AMP, FREQ = 0.75, 0.5  # reduced from 1.0 - peak velocity (pi rad/s) exceeded MPCs 3.0 rad/s constraint
def reference(t):
    theta_r = AMP * np.sin(2*np.pi*FREQ*t)
    theta_dot_r = AMP * 2*np.pi*FREQ * np.cos(2*np.pi*FREQ*t)
    theta_ddot_r = -AMP * (2*np.pi*FREQ)**2 * np.sin(2*np.pi*FREQ*t)
    return theta_r, theta_dot_r, theta_ddot_r

DT = 0.001
T_END = 5.0
STEPS = int(T_END / DT)

DIST_START, DIST_END, DIST_MAG = 1.0, 1.3, -80.0   # Stage 2 Task 8
NOISE_THETA_STD, NOISE_OMEGA_STD = 0.01, 0.05        # Stage 2 Task 8

def disturbance(t, present):
    if present and DIST_START <= t < DIST_END:
        return DIST_MAG
    return 0.0


def simulate_trial(controller, disturbance_present, noise_present, seed):
    """Run ONE trial for one controller under one condition.
    controller: 'pid', 'ff', or 'mpc'
    Returns a dict of measured metrics for this single trial."""
    rng = np.random.default_rng(seed)

    def measure(x_true):
        if not noise_present:
            return x_true.copy()
        noisy = x_true.copy()
        noisy[0, 0] += rng.normal(0, NOISE_THETA_STD)
        noisy[1, 0] += rng.normal(0, NOISE_OMEGA_STD)
        return noisy

    x_true = np.array([[0.0], [0.0]])
    prev_theta_meas = 0.0

    # MPC setup, only if needed for this trial (expensive to build)
    mpc = None
    if controller == 'mpc':
        model = do_mpc.model.Model('continuous')
        th = model.set_variable('_x', 'theta')
        thd = model.set_variable('_x', 'theta_dot')
        V = model.set_variable('_u', 'V')
        # For a moving reference, MPC needs a time-varying setpoint (tvp).
        # MUST be declared before model.setup() locks the model.
        theta_ref_tvp = model.set_variable('_tvp', 'theta_ref')
        model.set_rhs('theta', thd)
        model.set_rhs('theta_dot', A21*thd + B21*V)
        model.setup()

        mpc = do_mpc.controller.MPC(model)
        mpc.set_param(n_horizon=20, t_step=0.02, n_robust=0, store_full_solution=True)
        mpc.set_param(nlpsol_opts={'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0})
        mterm = (model.x['theta'] - theta_ref_tvp)**2
        lterm = (model.x['theta'] - theta_ref_tvp)**2
        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(V=0.01)
        mpc.bounds['lower', '_u', 'V'] = -100
        mpc.bounds['upper', '_u', 'V'] = 100
        mpc.bounds['lower', '_x', 'theta_dot'] = -3.0
        mpc.bounds['upper', '_x', 'theta_dot'] = 3.0

        tvp_template = mpc.get_tvp_template()
        def tvp_fun(t_now):
            for k in range(21):
                t_pred = t_now + k*0.02
                tvp_template['_tvp', k, 'theta_ref'] = AMP*np.sin(2*np.pi*FREQ*t_pred)
            return tvp_template
        mpc.set_tvp_fun(tvp_fun)
        mpc.setup()
        mpc.x0 = x_true
        mpc.set_initial_guess()

    t_log, theta_log, ref_log, u_log = [], [], [], []
    mpc_step_counter = 0
    u_mpc_last = 0.0
    peak_omega = 0.0

    for i in range(STEPS):
        t = i * DT
        x_meas = measure(x_true)
        theta_meas = x_meas[0, 0]
        theta_dot_meas = x_meas[1, 0]
        theta_r, theta_dot_r, theta_ddot_r = reference(t)

        if controller == 'pid':
            error = theta_r - theta_meas
            deriv = -(theta_meas - prev_theta_meas) / DT
            u = KP*error + KD*deriv
        elif controller == 'ff':
            error = theta_r - theta_meas
            deriv = -(theta_meas - prev_theta_meas) / DT
            u_fb = KP*error + KD*deriv
            u_ff = (R/K) * (J*theta_ddot_r + (b + K**2/R)*theta_dot_r)
            u = u_fb + u_ff
        elif controller == 'mpc':
            # MPC runs at 20ms (50Hz); hold last command between solves
            if i % 20 == 0:
                u_mpc_last = float(mpc.make_step(x_meas)[0, 0])
                mpc_step_counter += 1
            u = u_mpc_last
        else:
            raise ValueError(controller)

        prev_theta_meas = theta_meas

        dist = disturbance(t, disturbance_present)
        theta_ddot = A21*x_true[1,0] + B21*u + dist
        x_true[0,0] = x_true[0,0] + x_true[1,0]*DT
        x_true[1,0] = x_true[1,0] + theta_ddot*DT

        peak_omega = max(peak_omega, abs(x_true[1,0]))

        t_log.append(t); theta_log.append(x_true[0,0]); ref_log.append(theta_r); u_log.append(u)

    t_log = np.array(t_log); theta_log = np.array(theta_log)
    ref_log = np.array(ref_log); u_log = np.array(u_log)
    error_log = ref_log - theta_log

    rms_error = np.sqrt(np.mean(error_log**2))
    max_error = np.max(np.abs(error_log))
    chatter = np.std(np.diff(u_log))
    mean_abs_u = np.mean(np.abs(u_log))
    max_abs_u = np.max(np.abs(u_log))

    # Settling time (sinusoidal redefinition, TEST_PROTOCOL.md Task 18)
    final_window = t_log >= (T_END - T_END*0.2)
    steady_state_rms = np.sqrt(np.mean(error_log[final_window]**2))
    band = 1.5 * steady_state_rms
    settled_idx = None
    for idx in range(len(error_log)):
        if np.all(np.abs(error_log[idx:]) < band):
            settled_idx = idx
            break
    settling_time = t_log[settled_idx] if settled_idx is not None else None

    result = {
        'controller': controller,
        'disturbance': disturbance_present,
        'noise': noise_present,
        'seed': seed,
        'rms_error': rms_error,
        'max_error': max_error,
        'chatter': chatter,
        'mean_abs_u': mean_abs_u,
        'max_abs_u': max_abs_u,
        'settling_time': settling_time,
    }
    if controller == 'mpc':
        result['peak_omega'] = peak_omega
        result['constraint_violated'] = peak_omega > 3.0

    return result


if __name__ == '__main__':
    # Quick smoke test: one trial per controller, no disturbance/noise
    for ctrl in ['pid', 'ff', 'mpc']:
        r = simulate_trial(ctrl, disturbance_present=False, noise_present=False, seed=0)
        print(f"{ctrl}: rms_error={r['rms_error']:.5f}  max_error={r['max_error']:.5f}  "
              f"settling_time={r['settling_time']}  chatter={r['chatter']:.4f}")
