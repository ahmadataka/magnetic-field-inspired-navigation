# Dynamic Benchmark Master Summary

This file consolidates the currently available dynamic-environment benchmarks across the clean Python repo and Webots validation suites.

Included benchmark families:
- `dynamic_double_integrator_2d`
- `dynamic_differential_drive_2d`
- `dynamic_double_integrator_3d`
- `dynamic_quadrotor_3d`
- `webots_epuck_dynamic`
- `webots_crazyflie_dynamic`

Metric interpretation:
- `success_rate`: final-state success under the suite safety rule
- `goal_reached_once_rate`: fraction of runs that entered the goal region at least once without a safety violation
- `mean_time_to_goal_steps` and `mean_time_to_goal_seconds`: averaged over finite goal-reaching runs only
- `mean_min_clearance` and `worst_min_clearance`: obstacle-avoidance margin indicators
- `collision_rate` and `safety_violation_rate`: failure indicators

| Model | Method | N | Succ | Succ Rate | Reach Once | Path | T_goal (steps) | T_goal (s) | Final Err | Mean Clr | Worst Clr | Coll Rate | Safe Viol | Eff | Status |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 2D double integrator | APF | 6 | 4 | 0.667 | 0.667 | 31.346 | 1396.8 | 27.94 | 15.777 | 1.056 | 0.838 | 0.000 | 0.000 | 0.734 | completed |
| 2D double integrator | Haddadin | 6 | 2 | 0.333 | 0.333 | 10.175 | 1316.5 | 26.33 | 4.382 | 0.204 | -0.025 | 0.667 | 0.667 | 0.904 | completed |
| 2D double integrator | MFI-Geometric | 6 | 5 | 0.833 | 0.833 | 19.243 | 1134.2 | 22.68 | 0.181 | 1.048 | 0.483 | 0.000 | 0.000 | 0.695 | completed |
| 2D double integrator | MFI-PD | 6 | 5 | 0.833 | 0.833 | 18.562 | 1916.2 | 38.32 | 0.884 | 0.912 | 0.299 | 0.000 | 0.000 | 0.676 | completed |
| 2D double integrator | Sabattini | 6 | 0 | 0.000 | 0.000 | 5.977 | - | - | 6.962 | -0.006 | -0.010 | 1.000 | 1.000 | 1.000 | completed |
| 2D differential drive | APF | 6 | 5 | 0.833 | 0.833 | 18.770 | 3487.8 | 69.76 | 0.274 | 1.304 | 0.929 | 0.000 | 0.000 | 0.745 | completed |
| 2D differential drive | Haddadin | 6 | 5 | 0.833 | 0.833 | 12.092 | 3106.8 | 62.14 | 1.414 | 0.768 | -0.002 | 0.167 | 0.167 | 0.961 | completed |
| 2D differential drive | MFI-Geometric | 6 | 4 | 0.667 | 0.667 | 10.299 | 5516.2 | 110.33 | 2.770 | 2.851 | -0.007 | 0.167 | 0.167 | 0.967 | completed |
| 2D differential drive | MFI-PD | 6 | 4 | 0.667 | 0.667 | 11.848 | 4260.8 | 85.22 | 2.446 | 1.972 | -0.014 | 0.167 | 0.167 | 0.922 | completed |
| 2D differential drive | Sabattini | 6 | 3 | 0.500 | 0.500 | 9.299 | 2883.0 | 57.66 | 3.640 | 0.375 | -0.018 | 0.500 | 0.500 | 1.000 | completed |
| 3D double integrator | APF | 5 | 3 | 0.600 | 0.600 | 34.721 | 2123.0 | 42.46 | 20.531 | 1.820 | 1.321 | 0.000 | 0.000 | 0.726 | completed |
| 3D double integrator | Haddadin | 5 | 3 | 0.600 | 0.600 | 13.287 | 2833.8 | 56.68 | 1.610 | 0.631 | -0.009 | 0.400 | 0.400 | 0.833 | completed |
| 3D double integrator | MFI-Geometric | 5 | 3 | 0.600 | 0.600 | 17.259 | 2643.7 | 52.87 | 3.002 | 3.157 | -0.003 | 0.200 | 0.200 | 0.763 | completed |
| 3D double integrator | MFI-PD | 5 | 3 | 0.600 | 0.600 | 15.125 | 2639.3 | 52.79 | 2.145 | 2.251 | -0.009 | 0.200 | 0.200 | 0.756 | completed |
| 3D double integrator | Sabattini | 5 | 1 | 0.200 | 0.200 | 7.529 | 1458.0 | 29.16 | 4.541 | 0.123 | -0.020 | 0.800 | 0.800 | 1.000 | completed |
| 3D quadrotor | APF | 5 | 0 | 0.000 | 0.000 | 36.846 | - | - | 3.246 | 0.233 | -0.089 | 0.600 | 0.600 | 0.515 | completed |
| 3D quadrotor | Haddadin | 5 | 0 | 0.000 | 0.000 | 50.448 | - | - | 3.283 | 0.857 | -0.015 | 0.400 | 0.400 | 0.436 | completed |
| 3D quadrotor | MFI-Geometric | 5 | 2 | 0.400 | 0.400 | 24.132 | 1415.0 | 28.30 | 3.210 | 0.693 | -0.012 | 0.600 | 0.600 | 0.457 | completed |
| 3D quadrotor | MFI-PD | 5 | 0 | 0.000 | 0.200 | 61.397 | 1136.0 | 22.72 | 1.314 | 0.962 | -0.009 | 0.200 | 0.200 | 0.211 | completed |
| 3D quadrotor | Sabattini | 5 | 0 | 0.000 | 0.000 | 36.944 | - | - | 3.071 | 0.349 | -0.038 | 0.600 | 0.600 | 0.532 | completed |
| Webots e-puck | APF | 6 | 4 | 0.667 | 0.667 | 8.292 | 1460.5 | 93.47 | 0.737 | 0.399 | 0.128 | 0.000 | 0.000 | 0.175 | goal_reached |
| Webots e-puck | Haddadin | 6 | 5 | 0.833 | 0.833 | 0.990 | 482.3 | 30.87 | 0.300 | 0.224 | 0.007 | 0.000 | 0.167 | 0.831 | goal_reached |
| Webots e-puck | MFI-Geometric | 6 | 3 | 0.500 | 0.500 | 1.356 | 416.5 | 26.66 | 0.336 | 0.223 | -0.001 | 0.333 | 0.500 | 0.668 | goal_reached |
| Webots e-puck | MFI-PD | 6 | 5 | 0.833 | 0.833 | 1.392 | 511.4 | 32.73 | 0.324 | 0.249 | -0.001 | 0.167 | 0.167 | 0.589 | goal_reached |
| Webots e-puck | Sabattini | 6 | 1 | 0.167 | 0.167 | 2.857 | 688.5 | 44.06 | 1.658 | 0.159 | -0.001 | 0.167 | 0.667 | 0.725 | timeout |
| Webots Crazyflie | APF | 3 | 3 | 1.000 | 1.000 | 8.986 | 954.7 | 30.55 | 0.173 | 0.629 | 0.420 | 0.000 | 0.000 | 0.452 | goal_reached |
| Webots Crazyflie | Haddadin | 3 | 2 | 0.667 | 0.667 | 7.430 | 662.3 | 21.19 | 0.082 | 0.594 | -0.098 | 0.333 | 0.333 | 0.451 | goal_reached |
| Webots Crazyflie | MFI-Geometric | 3 | 3 | 1.000 | 1.000 | 9.094 | 1065.3 | 34.09 | 0.166 | 0.800 | 0.399 | 0.000 | 0.000 | 0.471 | goal_reached |
| Webots Crazyflie | MFI-PD | 3 | 3 | 1.000 | 1.000 | 6.192 | 766.0 | 24.51 | 0.169 | 0.663 | 0.346 | 0.000 | 0.000 | 0.541 | goal_reached |
| Webots Crazyflie | Sabattini | 3 | 1 | 0.333 | 0.333 | 5.776 | 427.0 | 13.66 | 0.993 | 0.349 | -0.135 | 0.333 | 0.333 | 0.462 | collision |
