import matplotlib
matplotlib.use('Agg')
import numpy as np
import control as ct
import matplotlib.pyplot as plt

# --- Same plant as Task 1 ---
J, b, K, R = 0.01, 0.1, 0.01, 1.0
A = np.array([[0, 1], [0, -(b + K**2/R)/J]])
B = np.array([[0], [K/(R*J)]])
C = np.array([[1, 0]])
D = np.array([[0]])
plant = ct.ss(A, B, C, D)

# --- Simulation settings ---
dt = 0.001          # 1kHz control loop — realistic for a servo
T_end = 3.0
steps = int(T_end / dt)
target = 1.0         # target shaft angle (rad)

# --- PID gains (starting point — we'll tune after seeing the response) ---
Kp, Ki, Kd = 144.0, 0.0, 6.79

# --- Discrete-time simulation state ---
x = np.array([[0.0], [0.0]])   # [theta, theta_dot]
integral = 0.0
prev_error = 0.0

t_log, theta_log, u_log = [], [], []

for i in range(steps):
    t = i * dt
    theta = x[0, 0]
    error = target - theta

    integral += error * dt
    derivative = (error - prev_error) / dt
    u = Kp*error + Ki*integral + Kd*derivative
    prev_error = error

    # Advance plant one step (simple Euler integration of x' = Ax + Bu)
    x_dot = A @ x + B * u
    x = x + x_dot * dt

    t_log.append(t)
    theta_log.append(theta)
    u_log.append(u)

t_log, theta_log, u_log = np.array(t_log), np.array(theta_log), np.array(u_log)

# --- Characterize step response ---
final_value = theta_log[-1]
overshoot = (max(theta_log) - final_value) / final_value * 100 if max(theta_log) > final_value else 0.0

# settling time: first time error stays within 2% of target for the rest of the run
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

# --- Plot ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(t_log, theta_log, label="θ (actual)")
ax1.axhline(target, color='r', linestyle='--', label="target")
ax1.set_ylabel("Shaft angle (rad)")
ax1.set_title("PID Baseline — Step Response")
ax1.legend()
ax1.grid(True)

ax2.plot(t_log, u_log, color='orange')
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Control effort (V)")
ax2.set_title("Control Input (Voltage)")
ax2.grid(True)

plt.tight_layout()
plt.savefig("stage1_plant/pid_baseline_response.png", dpi=120)
print("\nSaved plot: stage1_plant/pid_baseline_response.png")

# --- Diagnostic: compute actual closed-loop poles for these gains ---
# Open-loop: G(s) = 1/(s^2 + 10.01s), C(s) = Kp + Ki/s + Kd*s
s = ct.tf('s')
G = 1 / (s**2 + 10.01*s)
C = Kp + Ki/s + Kd*s
closed_loop = ct.feedback(C*G, 1)
poles = ct.poles(closed_loop)
print("\nClosed-loop poles:", poles)
for p in poles:
    print(f"  pole={p:.4f}  →  decay time constant ≈ {1/abs(p.real):.2f}s" if p.real != 0 else f"  pole={p}")
