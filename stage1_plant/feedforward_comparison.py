import matplotlib
matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt

J, b, K, R = 0.01, 0.1, 0.01, 1.0
Kp, Kd = 144.0, 6.79   # from Task 2 pole placement

dt = 0.001
T_end = 5.0
steps = int(T_end / dt)

# --- Moving reference: sinusoid ---
amp, freq = 1.0, 0.5  # rad, Hz
def ref(t):
    theta_r = amp * np.sin(2*np.pi*freq*t)
    theta_dot_r = amp * 2*np.pi*freq * np.cos(2*np.pi*freq*t)
    theta_ddot_r = -amp * (2*np.pi*freq)**2 * np.sin(2*np.pi*freq*t)
    return theta_r, theta_dot_r, theta_ddot_r

def simulate(use_feedforward):
    x = np.array([[0.0], [0.0]])
    t_log, theta_log, ref_log = [], [], []
    for i in range(steps):
        t = i * dt
        theta_r, theta_dot_r, theta_ddot_r = ref(t)
        theta, theta_dot = x[0,0], x[1,0]

        error = theta_r - theta
        error_dot = theta_dot_r - theta_dot
        u_fb = Kp*error + Kd*error_dot

        if use_feedforward:
            u_ff = (R/K) * (J*theta_ddot_r + (b + K**2/R)*theta_dot_r)
        else:
            u_ff = 0.0

        u = u_fb + u_ff

        A = np.array([[0,1],[0,-(b+K**2/R)/J]])
        B = np.array([[0],[K/(R*J)]])
        x_dot = A @ x + B*u
        x = x + x_dot*dt

        t_log.append(t); theta_log.append(theta); ref_log.append(theta_r)
    return np.array(t_log), np.array(theta_log), np.array(ref_log)

t, theta_fb_only, ref_sig = simulate(use_feedforward=False)
_, theta_fb_ff, _ = simulate(use_feedforward=True)

err_fb_only = ref_sig - theta_fb_only
err_fb_ff = ref_sig - theta_fb_ff

rms_fb_only = np.sqrt(np.mean(err_fb_only**2))
rms_fb_ff = np.sqrt(np.mean(err_fb_ff**2))
max_fb_only = np.max(np.abs(err_fb_only))
max_fb_ff = np.max(np.abs(err_fb_ff))

print(f"Feedback-only    → RMS error: {rms_fb_only:.5f} rad | Max error: {max_fb_only:.5f} rad")
print(f"Feedback+FF      → RMS error: {rms_fb_ff:.5f} rad | Max error: {max_fb_ff:.5f} rad")
print(f"RMS error reduction: {(1 - rms_fb_ff/rms_fb_only)*100:.1f}%")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8))
ax1.plot(t, ref_sig, 'r--', label='reference', linewidth=1)
ax1.plot(t, theta_fb_only, label='feedback-only', alpha=0.8)
ax1.plot(t, theta_fb_ff, label='feedback+feedforward', alpha=0.8)
ax1.set_ylabel("θ (rad)"); ax1.set_title("Tracking: Feedback-only vs. Feedback+Feedforward")
ax1.legend(); ax1.grid(True)

ax2.plot(t, err_fb_only, label='feedback-only error')
ax2.plot(t, err_fb_ff, label='feedback+FF error')
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Tracking error (rad)")
ax2.legend(); ax2.grid(True)

plt.tight_layout()
plt.savefig("stage1_plant/feedforward_comparison.png", dpi=120)
print("\nSaved plot: stage1_plant/feedforward_comparison.png")
