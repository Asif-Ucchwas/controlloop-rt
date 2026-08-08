"""
Stage 5 Task 20: Comparison plots and tables from the 600-trial benchmark.
Reads benchmark_summary.csv (Task 19 output).
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import csv

with open('stage5_benchmarking/benchmark_summary.csv') as f:
    rows = list(csv.DictReader(f))

CONTROLLERS = ['pid', 'ff', 'mpc']
CONTROLLER_LABELS = {'pid': 'PID', 'ff': 'PD+Feedforward', 'mpc': 'MPC'}
CONDITIONS = ['no_dist_no_noise', 'no_dist_noise', 'dist_no_noise', 'dist_noise']
CONDITION_LABELS = ['No dist.\nNo noise', 'No dist.\nNoise', 'Dist.\nNo noise', 'Dist.\nNoise']
COLORS = {'pid': '#d15642', 'ff': '#1f9e89', 'mpc': '#3f6ea8'}

def get(controller, condition, field):
    for r in rows:
        if r['controller'] == controller and r['condition'] == condition:
            val = r[field]
            return float(val) if val not in ('', None) else np.nan
    return np.nan

def grouped_bar_plot(field_mean, field_std, title, ylabel, filename, log_scale=False):
    x = np.arange(len(CONDITIONS))
    width = 0.25
    fig, ax = plt.subplots(figsize=(9, 6))
    for i, ctrl in enumerate(CONTROLLERS):
        means = [get(ctrl, c, field_mean) for c in CONDITIONS]
        stds = [get(ctrl, c, field_std) for c in CONDITIONS]
        ax.bar(x + (i-1)*width, means, width, yerr=stds, capsize=4,
               label=CONTROLLER_LABELS[ctrl], color=COLORS[ctrl], alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(CONDITION_LABELS)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if log_scale:
        ax.set_yscale('log')
    ax.legend()
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'stage5_benchmarking/{filename}', dpi=120)
    plt.close()
    print(f"Saved: stage5_benchmarking/{filename}")

grouped_bar_plot('rms_error_mean', 'rms_error_std',
                  'RMS Tracking Error — 50-Trial Mean ± Std (600 trials total)',
                  'RMS error (rad)', 'rms_error_comparison.png')

grouped_bar_plot('max_error_mean', 'max_error_std',
                  'Max Tracking Error — 50-Trial Mean ± Std',
                  'Max error (rad)', 'max_error_comparison.png')

grouped_bar_plot('settling_time_mean', 'settling_time_std',
                  'Settling Time (sinusoidal-redefined) — 50-Trial Mean ± Std',
                  'Settling time (s)', 'settling_time_comparison.png')

grouped_bar_plot('chatter_mean', 'chatter_std',
                  'Control Effort Chatter — 50-Trial Mean ± Std (log scale)',
                  'Chatter (std of consecutive ΔV)', 'chatter_comparison.png',
                  log_scale=True)

# MPC-only: constraint violation rate
fig, ax = plt.subplots(figsize=(7, 5))
violation_rates = [get('mpc', c, 'constraint_violation_rate_pct') for c in CONDITIONS]
bars = ax.bar(CONDITION_LABELS, violation_rates, color=COLORS['mpc'], alpha=0.85)
for bar, rate in zip(bars, violation_rates):
    ax.text(bar.get_x() + bar.get_width()/2, rate + 2, f'{rate:.0f}%',
            ha='center', fontweight='bold')
ax.set_ylabel('Trials with velocity constraint violated (%)')
ax.set_title('MPC Constraint Violation Rate by Condition (out of 50 trials each)')
ax.set_ylim(0, 110)
ax.grid(True, axis='y', alpha=0.3)
plt.tight_layout()
plt.savefig('stage5_benchmarking/mpc_constraint_violation.png', dpi=120)
plt.close()
print("Saved: stage5_benchmarking/mpc_constraint_violation.png")

# --- Markdown results table for the paper/README ---
with open('stage5_benchmarking/RESULTS_TABLE.md', 'w') as f:
    f.write("# Benchmark Results Summary (600 trials: 3 controllers x 4 conditions x 50 trials)\n\n")
    f.write("| Controller | Condition | RMS error | Max error | Settling (s) | Chatter | MPC constraint violation |\n")
    f.write("|---|---|---|---|---|---|---|\n")
    for ctrl in CONTROLLERS:
        for cond, cond_label in zip(CONDITIONS, ['No dist/No noise', 'No dist/Noise', 'Dist/No noise', 'Dist/Noise']):
            rms_m = get(ctrl, cond, 'rms_error_mean'); rms_s = get(ctrl, cond, 'rms_error_std')
            max_m = get(ctrl, cond, 'max_error_mean'); max_s = get(ctrl, cond, 'max_error_std')
            settle_m = get(ctrl, cond, 'settling_time_mean')
            chatter_m = get(ctrl, cond, 'chatter_mean')
            viol = get(ctrl, cond, 'constraint_violation_rate_pct')
            viol_str = f"{viol:.0f}%" if not np.isnan(viol) else "N/A"
            f.write(f"| {CONTROLLER_LABELS[ctrl]} | {cond_label} | "
                    f"{rms_m:.4f} ± {rms_s:.4f} | {max_m:.4f} ± {max_s:.4f} | "
                    f"{settle_m:.3f} | {chatter_m:.2f} | {viol_str} |\n")

print("Saved: stage5_benchmarking/RESULTS_TABLE.md")
print("\nTask 20 complete: 5 plots + 1 results table generated.")
