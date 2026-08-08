# Benchmark Results Summary (600 trials: 3 controllers x 4 conditions x 50 trials)

| Controller | Condition | RMS error | Max error | Settling (s) | Chatter | MPC constraint violation |
|---|---|---|---|---|---|---|
| PID | No dist/No noise | 0.1926 ± 0.0000 | 0.2770 ± 0.0000 | 0.000 | 0.06 | N/A |
| PID | No dist/Noise | 0.1926 ± 0.0001 | 0.2780 ± 0.0006 | 0.000 | 167.67 | N/A |
| PID | Dist/No noise | 0.1940 ± 0.0000 | 0.3744 ± 0.0000 | 1.370 | 0.13 | N/A |
| PID | Dist/Noise | 0.1940 ± 0.0001 | 0.3744 ± 0.0006 | 1.370 | 167.67 | N/A |
| PD+Feedforward | No dist/No noise | 0.0783 ± 0.0000 | 0.1433 ± 0.0000 | 0.272 | 0.05 | N/A |
| PD+Feedforward | No dist/Noise | 0.0783 ± 0.0001 | 0.1432 ± 0.0008 | 0.272 | 167.67 | N/A |
| PD+Feedforward | Dist/No noise | 0.1205 ± 0.0000 | 0.4806 ± 0.0000 | 1.466 | 0.13 | N/A |
| PD+Feedforward | Dist/Noise | 0.1205 ± 0.0001 | 0.4806 ± 0.0006 | 1.466 | 167.67 | N/A |
| MPC | No dist/No noise | 0.0703 ± 0.0000 | 0.2691 ± 0.0000 | 1.099 | 0.27 | 0% |
| MPC | No dist/Noise | 0.0703 ± 0.0005 | 0.2689 ± 0.0012 | 1.505 | 0.27 | 0% |
| MPC | Dist/No noise | 0.0966 ± 0.0000 | 0.3035 ± 0.0000 | 2.418 | 1.81 | 100% |
| MPC | Dist/Noise | 0.0952 ± 0.0066 | 0.3151 ± 0.0313 | 2.578 | 1.76 | 100% |
