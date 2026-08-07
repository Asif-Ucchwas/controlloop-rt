import matplotlib
matplotlib.use('Agg')
import numpy as np
import do_mpc

J, b, K, R = 0.01, 0.1, 0.01, 1.0
target = 1.0
V_MAX = 100.0
OMEGA_MAX = 3.0

model = do_mpc.model.Model('continuous')
theta_v = model.set_variable('_x', 'theta')
theta_dot_v = model.set_variable('_x', 'theta_dot')
V = model.set_variable('_u', 'V')
model.set_rhs('theta', theta_dot_v)
model.set_rhs('theta_dot', -(b + K**2/R)/J * theta_dot_v + K/(R*J) * V)
model.setup()

mpc = do_mpc.controller.MPC(model)
mpc.set_param(n_horizon=5, t_step=0.02, n_robust=0, store_full_solution=True)
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
simulator.set_param(t_step=0.02)
simulator.setup()
estimator = do_mpc.estimator.StateFeedback(model)
x0 = np.array([[0.0], [0.0]])
mpc.x0 = x0; simulator.x0 = x0
mpc.set_initial_guess()

theta_log = []
for i in range(150):
    u0 = mpc.make_step(x0)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)
    theta_log.append(x0[0,0])

theta_log = np.array(theta_log)
print(f"theta at t=1s (step 50):  {theta_log[49]:.5f}")
print(f"theta at t=2s (step 100): {theta_log[99]:.5f}")
print(f"theta at t=3s (step 149): {theta_log[149]:.5f}")
print(f"min theta: {theta_log.min():.5f}   max theta: {theta_log.max():.5f}")
print(f"last 10 values: {theta_log[-10:]}")
