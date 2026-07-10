# Dynamic Paper Figures

These are the publication-oriented trajectory figures extracted from the larger dynamic benchmark suite.

Design choices:
- only three algorithms are shown in each figure: `MFI-PD`, `MFI-Geometric`, and `APF`.
- moving obstacles are drawn using three snapshots: initial, mid-interaction, and near-end.
- dashed arrows indicate the obstacle motion direction.
- the 3D cases are shown as `x-y` and `x-z` projections for readability.

## Dynamic 3D double-integrator

- scenario: `moving_prism_gate_3d`
- description: A moving convex prism sweeps through the route while a second obstacle drifts downstream.
- PNG: [figure_dynamic_double_integrator_3d.png](</Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation/artifacts/summaries/dynamic_paper_figures/figure_dynamic_double_integrator_3d.png>)
- PDF: [figure_dynamic_double_integrator_3d.pdf](</Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation/artifacts/summaries/dynamic_paper_figures/figure_dynamic_double_integrator_3d.pdf>)

## Webots e-puck dynamic benchmark

- scenario: `moving_u_shape`
- description: A non-convex U-shape whose mouth faces the robot while the whole obstacle drifts downward during the encounter. Rescaled by 0.132 for Webots e-puck arena.
- PNG: [figure_dynamic_webots_epuck.png](</Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation/artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_epuck.png>)
- PDF: [figure_dynamic_webots_epuck.pdf](</Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation/artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_epuck.pdf>)

## Webots Crazyflie dynamic benchmark

- scenario: `moving_sphere_crossing_3d`
- description: A moving sphere crosses the direct route in 3D. Uniformly scaled by 0.291 and lifted by 0.496 m for Webots Crazyflie.
- PNG: [figure_dynamic_webots_crazyflie.png](</Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation/artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_crazyflie.png>)
- PDF: [figure_dynamic_webots_crazyflie.pdf](</Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation/artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_crazyflie.pdf>)
