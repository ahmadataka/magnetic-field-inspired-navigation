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
| 2D differential drive | MFI-Geometric | 6 | 4 | 0.667 | 0.667 | 11.686 | 3514.2 | 70.28 | 1.992 | 2.258 | -0.005 | 0.167 | 0.167 | 0.914 | completed |
| 2D differential drive | MFI-PD | 6 | 3 | 0.500 | 0.500 | 11.908 | 3133.3 | 62.67 | 3.094 | 1.222 | -0.002 | 0.333 | 0.333 | 0.894 | completed |
| 2D differential drive | Sabattini | 6 | 3 | 0.500 | 0.500 | 9.299 | 2883.0 | 57.66 | 3.640 | 0.375 | -0.018 | 0.500 | 0.500 | 1.000 | completed |
| 3D double integrator | APF | 5 | 0 | 0.000 | 0.000 | 127748.078 | - | - | 12.561 | -0.285 | -0.657 | 1.000 | 1.000 | 0.706 | completed |
| 3D double integrator | Haddadin | 5 | 1 | 0.200 | 0.400 | 187.255 | 3528.0 | 70.56 | 4.348 | 0.264 | -0.141 | 0.600 | 0.600 | 0.404 | completed |
| 3D double integrator | MFI-Geometric | 5 | 3 | 0.600 | 0.600 | 12.307 | 2022.0 | 40.44 | 2.159 | 2.083 | -0.020 | 0.400 | 0.400 | 0.858 | completed |
| 3D double integrator | MFI-PD | 5 | 1 | 0.200 | 0.200 | 46.689 | 1514.0 | 30.28 | 7.088 | 0.469 | -0.037 | 0.600 | 0.600 | 0.706 | completed |
| 3D double integrator | Sabattini | 5 | 1 | 0.200 | 0.200 | 6.575 | 941.0 | 18.82 | 5.494 | 0.017 | -0.037 | 0.800 | 0.800 | 1.000 | completed |
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
| Webots Crazyflie | APF | 3 | 1 | 0.333 | 0.333 | 9.908 | 1250.0 | 40.00 | 3.447 | 0.535 | 0.143 | 0.000 | 0.000 | 0.418 | collision |
| Webots Crazyflie | Haddadin | 3 | 2 | 0.667 | 0.667 | 1880.624 | 667.0 | 21.34 | 1612.438 | 0.659 | -0.051 | 0.333 | 0.333 | 0.626 | goal_reached |
| Webots Crazyflie | MFI-Geometric | 3 | 2 | 0.667 | 0.667 | 17.446 | 1179.0 | 37.73 | 0.788 | 0.705 | 0.332 | 0.000 | 0.000 | 0.257 | goal_reached |
| Webots Crazyflie | MFI-PD | 3 | 1 | 0.333 | 0.333 | 16.091 | 1826.0 | 58.43 | 0.386 | 0.723 | 0.334 | 0.000 | 0.000 | 0.192 | collision |
| Webots Crazyflie | Sabattini | 3 | 1 | 0.333 | 0.333 | 15.362 | 562.0 | 17.98 | 0.898 | 0.455 | -0.100 | 0.333 | 0.333 | 0.281 | collision |
