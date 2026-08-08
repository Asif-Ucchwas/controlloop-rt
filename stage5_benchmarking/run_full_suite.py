"""
Stage 5 Task 19: Full 600-trial benchmark suite.
Protocol: stage5_benchmarking/TEST_PROTOCOL.md
Instrumentation: stage5_benchmarking/run_benchmark.py

Writes results incrementally (crash-safe) to benchmark_results.csv,
then aggregates to benchmark_summary.csv at the end.
"""
import csv
import time
import numpy as np
from run_benchmark import simulate_trial

CONDITIONS = [
    (False, False, 'no_dist_no_noise'),
    (False, True,  'no_dist_noise'),
    (True,  False, 'dist_no_noise'),
    (True,  True,  'dist_noise'),
]
CONTROLLERS = ['pid', 'ff', 'mpc']
N_TRIALS = 50

RAW_FIELDNAMES = ['controller', 'condition', 'disturbance', 'noise', 'seed',
                  'rms_error', 'max_error', 'chatter', 'mean_abs_u', 'max_abs_u',
                  'settling_time', 'peak_omega', 'constraint_violated']

total_runs = len(CONTROLLERS) * len(CONDITIONS) * N_TRIALS
run_idx = 0
start_time = time.time()

print(f"Starting full benchmark suite: {total_runs} total runs "
      f"({len(CONTROLLERS)} controllers x {len(CONDITIONS)} conditions x {N_TRIALS} trials)")
print("Using common random seeds (0-49) across controllers within each condition, "
      "per common-random-numbers methodology (TEST_PROTOCOL.md).\n")

all_results = []

with open('stage5_benchmarking/benchmark_results.csv', 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=RAW_FIELDNAMES, extrasaction='ignore')
    writer.writeheader()

    for dist, noise, cond_name in CONDITIONS:
        for trial in range(N_TRIALS):
            seed = trial   # SAME seed across controllers for this trial index
            for controller in CONTROLLERS:
                result = simulate_trial(controller, dist, noise, seed=seed)
                result['condition'] = cond_name
                row = {k: result.get(k, '') for k in RAW_FIELDNAMES}
                writer.writerow(row)
                all_results.append(result)
                run_idx += 1

            if run_idx % 30 == 0:
                elapsed = time.time() - start_time
                rate = run_idx / elapsed
                remaining = (total_runs - run_idx) / rate if rate > 0 else 0
                print(f"Progress: {run_idx}/{total_runs}  "
                      f"({elapsed:.0f}s elapsed, ~{remaining:.0f}s remaining)  "
                      f"[{cond_name}, trial {trial+1}/{N_TRIALS}]")
                f.flush()

elapsed_total = time.time() - start_time
print(f"\nAll {total_runs} runs complete in {elapsed_total:.0f}s "
      f"({elapsed_total/60:.1f} min). Raw results: benchmark_results.csv")

# --- Aggregate to summary ---
summary_rows = []
for controller in CONTROLLERS:
    for dist, noise, cond_name in CONDITIONS:
        subset = [r for r in all_results
                  if r['controller'] == controller and r['disturbance'] == dist
                  and r['noise'] == noise]

        rms_vals = [r['rms_error'] for r in subset]
        max_vals = [r['max_error'] for r in subset]
        chatter_vals = [r['chatter'] for r in subset]
        settling_vals = [r['settling_time'] for r in subset if r['settling_time'] is not None]
        never_settled = len(subset) - len(settling_vals)

        row = {
            'controller': controller,
            'condition': cond_name,
            'n_trials': len(subset),
            'rms_error_mean': np.mean(rms_vals),
            'rms_error_std': np.std(rms_vals),
            'max_error_mean': np.mean(max_vals),
            'max_error_std': np.std(max_vals),
            'chatter_mean': np.mean(chatter_vals),
            'chatter_std': np.std(chatter_vals),
            'settling_time_mean': np.mean(settling_vals) if settling_vals else '',
            'settling_time_std': np.std(settling_vals) if settling_vals else '',
            'never_settled_pct': 100 * never_settled / len(subset),
            'constraint_violation_rate_pct': '',
        }
        if controller == 'mpc':
            violated = [r.get('constraint_violated', False) for r in subset]
            row['constraint_violation_rate_pct'] = 100 * sum(violated) / len(violated)

        summary_rows.append(row)

summary_fieldnames = list(summary_rows[0].keys())
with open('stage5_benchmarking/benchmark_summary.csv', 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
    writer.writeheader()
    for row in summary_rows:
        writer.writerow(row)

print("Summary written: benchmark_summary.csv\n")
print("--- Quick Summary ---")
for row in summary_rows:
    print(f"{row['controller']:>4} | {row['condition']:>16} | "
          f"RMS={row['rms_error_mean']:.4f}+/-{row['rms_error_std']:.4f} | "
          f"max={row['max_error_mean']:.4f} | "
          f"settle={row['settling_time_mean']}")
