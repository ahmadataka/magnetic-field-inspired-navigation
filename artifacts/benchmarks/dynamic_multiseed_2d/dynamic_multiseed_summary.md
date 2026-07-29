# Dynamic Multiseed 2D Summary

This benchmark repeats the dynamic 2D scenarios with small seeded perturbations around the same nominal setup.

Perturbations per seed:
- start position lateral offset
- small initial velocity / heading perturbation
- small moving-obstacle time-phase shift

- seeds: `[0, 1, 2, 3, 4]`
- lateral offset scale: `0.12` m
- initial speed scale: `0.08` m/s
- obstacle time-shift scale: `0.35` s

| Scenario | Method | Success Rate | Collision Rate | Safety Viol Rate | Final Error | Min Clearance | Worst Clearance | T_goal (steps) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| moving_circle_crossing | MFI-PD | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.873 ± 0.259 | 0.483 | 1371.0 ± 35.5 |
| moving_circle_crossing | MFI-Geometric | 1.000 | 0.000 | 0.000 | 0.034 ± 0.001 | 2.218 ± 0.202 | 2.002 | 826.4 ± 9.6 |
| moving_circle_crossing | APF | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.047 ± 0.022 | 1.023 | 1254.4 ± 140.2 |
| moving_circle_crossing | Haddadin | 0.000 | 1.000 | 1.000 | 6.415 ± 0.317 | -0.152 ± 0.150 | -0.368 | - |
| moving_circle_crossing | Sabattini | 0.800 | 0.200 | 0.200 | 1.478 ± 2.968 | 1.026 ± 0.584 | -0.001 | 1040.2 ± 17.0 |
| moving_convex_sweeper | MFI-PD | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.733 ± 0.072 | 1.605 | 2127.0 ± 86.6 |
| moving_convex_sweeper | MFI-Geometric | 1.000 | 0.000 | 0.000 | 0.140 ± 0.024 | 0.986 ± 0.226 | 0.858 | 1252.8 ± 64.4 |
| moving_convex_sweeper | APF | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.056 ± 0.011 | 1.044 | 1362.6 ± 34.7 |
| moving_convex_sweeper | Haddadin | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.485 ± 0.090 | 0.333 | 1516.4 ± 49.8 |
| moving_convex_sweeper | Sabattini | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.049 ± 0.284 | 0.882 | 1336.8 ± 100.0 |
| moving_u_shape | MFI-PD | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.649 ± 0.105 | 1.584 | 2020.8 ± 496.1 |
| moving_u_shape | MFI-Geometric | 1.000 | 0.000 | 0.000 | 0.130 ± 0.012 | 0.467 ± 0.088 | 0.311 | 1306.4 ± 105.4 |
| moving_u_shape | APF | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.145 ± 0.054 | 1.053 | 2046.4 ± 495.9 |
| moving_u_shape | Haddadin | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.722 ± 0.014 | 0.699 | 1107.8 ± 12.2 |
| moving_u_shape | Sabattini | 0.200 | 0.800 | 0.800 | 8.026 ± 4.671 | 0.172 ± 0.393 | -0.007 | 1375.0 ± 0.0 |
| head_on_circle | MFI-PD | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.148 ± 0.116 | 1.068 | 1235.2 ± 33.3 |
| head_on_circle | MFI-Geometric | 1.000 | 0.000 | 0.000 | 0.027 ± 0.000 | 0.706 ± 0.031 | 0.681 | 870.6 ± 10.7 |
| head_on_circle | APF | 0.200 | 0.000 | 0.000 | 74.533 ± 41.582 | 0.842 ± 0.007 | 0.838 | 1278.0 ± 0.0 |
| head_on_circle | Haddadin | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 0.213 ± 0.031 | 0.187 | 1012.6 ± 8.9 |
| head_on_circle | Sabattini | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 0.489 ± 0.050 | 0.406 | 1003.0 ± 7.3 |
| wandering_circle | MFI-PD | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 0.444 ± 0.344 | 0.119 | 2315.6 ± 306.1 |
| wandering_circle | MFI-Geometric | 1.000 | 0.000 | 0.000 | 0.117 ± 0.010 | 0.640 ± 0.031 | 0.600 | 1149.8 ± 82.4 |
| wandering_circle | APF | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.039 ± 0.145 | 0.852 | 1265.0 ± 145.3 |
| wandering_circle | Haddadin | 0.200 | 0.600 | 0.800 | 3.813 ± 3.393 | -0.372 ± 0.405 | -0.786 | 1480.0 ± 53.7 |
| wandering_circle | Sabattini | 0.800 | 0.200 | 0.200 | 1.547 ± 3.122 | 0.301 ± 0.184 | -0.003 | 1050.5 ± 26.3 |
| mixed_static_dynamic_field | MFI-PD | 0.000 | 0.000 | 0.000 | 0.718 ± 0.028 | 0.769 ± 0.490 | 0.248 | - |
| mixed_static_dynamic_field | MFI-Geometric | 0.000 | 0.000 | 0.000 | 0.693 ± 0.002 | 0.879 ± 0.171 | 0.641 | - |
| mixed_static_dynamic_field | APF | 0.000 | 0.000 | 0.000 | 0.734 ± 0.000 | 1.057 ± 0.035 | 1.009 | - |
| mixed_static_dynamic_field | Haddadin | 0.000 | 0.800 | 1.000 | 3.770 ± 2.881 | -0.038 ± 0.059 | -0.125 | 2225.0 ± 0.0 |
| mixed_static_dynamic_field | Sabattini | 0.000 | 0.000 | 0.000 | 0.903 ± 0.117 | 1.000 ± 0.283 | 0.496 | - |
