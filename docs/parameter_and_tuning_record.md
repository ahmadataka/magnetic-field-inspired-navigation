# Parameter and Tuning Record

This document freezes the main benchmark-side parameter choices currently used in `magnetic-field-inspired-navigation`.

It is intended to answer two questions:

1. Which parameters are part of the core algorithm/model implementation?
2. Which parameters were tuned at the benchmark or simulator interface layer?

Whenever we make a major tuning change that affects reported results, this file should be updated together with the benchmark outputs.

## 0. Tuning protocol status

This section records the current fairness status of the dynamic benchmarks.

### 0.1 Current status as of 2026-07-30

- Core MFI equations are shared across static and dynamic benchmarks.
- Some model-facing and simulator-facing parameters are tuned separately at the benchmark layer.
- The strongest formal multi-seed evidence currently exists for the 2D double-integrator dynamic setting.
- The dynamic differential-drive setting is still being improved and should be described honestly as a harder embodied/nonholonomic case.

### 0.2 Current frozen-policy note

For the current dynamic differential-drive benchmark snapshot:

- tuning was performed only at the benchmark/model-interface layer,
- no change was made to the MFI field, goal-relaxation structure, or local-sensing logic,
- tuning focused on the hardest dynamic differential-drive scenes:
  - `head_on_circle`
  - `moving_u_shape`
  - `mixed_static_dynamic_field`
- the same frozen dynamic diff-drive profile is then evaluated across the full six-scenario suite.

### 0.3 Validation and holdout honesty note

At the moment, the dynamic differential-drive benchmark does **not** yet have a strict held-out tuning split.

That means:

- parameter values are now frozen and documented,
- but the current diff-drive dynamic results should still be framed as benchmark-development results rather than a fully holdout-validated final comparison.

For the paper, this should be stated clearly if the differential-drive dynamic table is discussed in detail.

## 1. Core configuration defaults

Primary source:

- `src/mfinav/config/simulation.py`

Important global defaults:

| Parameter | Default |
| --- | ---: |
| `dt` | `0.02` |
| `steps` | `6000` |
| `kp_goal` | `0.04` |
| `kd_goal` | `0.5` |
| `kd_speed` | `0.5` |
| `kp_goal_relaxed` | `0.04` |
| `kp_geom` | `5.0` |
| `speed_limit` | `1.5` |
| `magni_bound` | `2.5` |
| `r_l` | `4.0` |
| `r_la` | `2.5` |
| `c_field` | `12.0` |
| `c_perp` | `35.0` |
| `delta_r` | `0.5` |
| `epsilon_current` | `3e-6` |
| `goal_relaxation` | `True` |
| `goal_relaxation_mode` | `"legacy"` |
| `sensing_mode` | `"analytic"` |
| `sensor_range` | `6.0` |

Differential-drive projection defaults:

| Parameter | Default |
| --- | ---: |
| `max_linear_speed` | `1.0` |
| `max_angular_speed` | `2.5` |
| `speed_gain` | `0.35` |
| `heading_gain` | `2.5` |
| `min_forward_factor` | `0.2` |

## 2. Paper-style MFI reference configs

Primary source:

- `src/mfinav/config/simulation.py`

### 2.1 MFI-PD 2D

Created by `make_paper_pd_config()`:

| Parameter | Value |
| --- | ---: |
| `kp_goal` | `0.08` |
| `kd_goal` | `0.5` |
| `kd_speed` | `0.5` |
| `kp_goal_relaxed` | `0.08` |
| `kp_geom` | `5.0` |
| `speed_limit` | `1.5` |
| `magni_bound` | `2.5` |
| `r_l` | `3.0` |
| `r_la` | `2.0` |
| `c_field` | `10.0` |
| `c_perp` | `20.0` |
| `delta_r` | `0.5` |
| `epsilon_current` | `3e-6` |
| `goal_mode` | `"pd"` |
| `field_mode` | `"paper"` |
| `sensing_mode` | `"raycast"` |
| `goal_relaxation_mode` | `"paper"` |
| `max_acceleration` | `inf` |
| `max_speed_norm` | `inf` |

### 2.2 MFI-Geometric 2D

Created by `make_paper_geometric_config()`:

- same as MFI-PD 2D, except `goal_mode = "geometric"`

### 2.3 MFI-PD 3D

Created by `make_paper_pd_3d_config()`:

| Parameter | Value |
| --- | ---: |
| `sensing_mode` | `"analytic"` |
| `r_l` | `4.0` |
| `r_la` | `2.0` |
| `c_field` | `15.0` |
| `c_perp` | `20.0` |
| `speed_limit` | `1.0` |
| `kp_goal` | `0.08` |
| `kp_goal_relaxed` | `0.08` |

### 2.4 MFI-Geometric 3D

Created by `make_paper_geometric_3d_config()`:

| Parameter | Value |
| --- | ---: |
| `goal_mode` | `"geometric"` |
| `speed_limit` | `0.5` |
| `kp_goal` | `0.12` |
| `kp_goal_relaxed` | `0.06` |
| `kd_goal` | `0.5` |
| `kp_geom` | `0.3` |
| `c_field` | `22.0` |
| `c_perp` | `20.0` |
| `r_l` | `4.0` |
| `r_la` | `2.0` |
| `max_acceleration` | `4.0` |
| `max_speed_norm` | `2.0` |

## 3. Benchmark-side overrides

These do not change the core MFI equations. They only adapt the same guidance law to a specific model, simulator, or benchmark environment.

### 3.1 Differential-drive static benchmark

Primary source:

- `scripts/run_benchmarks_diff_drive.py`

Applied to all methods:

| Parameter | Value |
| --- | ---: |
| `max_linear_speed` | `1.2` |
| `max_angular_speed` | `3.0` |
| `speed_gain` | `1.5` |
| `heading_gain` | `4.0` |
| `min_forward_factor` | `0.25` |

### 3.2 Differential-drive dynamic benchmark

Primary source:

- `scripts/run_benchmarks_dynamic_diff_drive_2d.py`

Baseline methods (`apf`, `haddadin`, `sabattini`) use:

| Parameter | Value |
| --- | ---: |
| `max_linear_speed` | `1.2` |
| `max_angular_speed` | `3.0` |
| `speed_gain` | `1.5` |
| `heading_gain` | `4.0` |
| `min_forward_factor` | `0.25` |

Dynamic MFI-PD benchmark profile uses:

| Parameter | Value |
| --- | ---: |
| `max_linear_speed` | `0.85` |
| `max_angular_speed` | `4.8` |
| `speed_gain` | `1.0` |
| `heading_gain` | `6.5` |
| `min_forward_factor` | `0.02` |

Dynamic MFI-Geometric benchmark profile uses:

| Parameter | Value |
| --- | ---: |
| `max_linear_speed` | `0.75` |
| `max_angular_speed` | `5.2` |
| `speed_gain` | `0.9` |
| `heading_gain` | `7.0` |
| `min_forward_factor` | `0.0` |

Reason for this override:

- dynamic head-on scenes were limited by the differential-drive tracking layer forcing too much forward motion during large heading corrections
- the change is benchmark-side only, and does not alter the MFI field, goal relaxation, or local sensing equations

### 3.3 Double-integrator dynamic 3D benchmark

Primary source:

- `scripts/run_benchmarks_dynamic_3d.py`

This benchmark now uses a frozen benchmark-side 3D tuning profile for the MFI variants.

Dynamic MFI-PD 3D benchmark profile uses:

| Parameter | Value |
| --- | ---: |
| `kp_goal` | `0.06` |
| `kp_goal_relaxed` | `0.06` |
| `kd_goal` | `0.55` |
| `speed_limit` | `0.75` |
| `r_l` | `5.0` |
| `r_la` | `3.0` |
| `c_field` | `20.0` |
| `c_perp` | `24.0` |
| `max_acceleration` | `3.2` |
| `max_speed_norm` | `1.6` |

Dynamic MFI-Geometric 3D benchmark profile uses:

| Parameter | Value |
| --- | ---: |
| `speed_limit` | `0.42` |
| `kp_goal` | `0.10` |
| `kp_goal_relaxed` | `0.05` |
| `kp_geom` | `0.34` |
| `r_l` | `5.0` |
| `r_la` | `3.0` |
| `c_field` | `24.0` |
| `c_perp` | `24.0` |
| `max_acceleration` | `3.5` |
| `max_speed_norm` | `1.8` |

Reason for this override:

- the original 3D benchmark was too brittle in dynamic crossing and wandering scenes
- increasing activation radius and moderating guidance-speed balance substantially improved 3D dynamic performance without changing the 3D MFI equations themselves
- the hardest pure head-on sphere case is still not solved by this benchmark-side tuning alone

### 3.4 Webots e-puck dynamic benchmark

Primary source:

- `scripts/run_webots_epuck_dynamic_benchmarks.py`

Method-specific overrides currently frozen there:

#### `paper_pd`

| Parameter | Value |
| --- | ---: |
| `kp_goal` | `0.16` |
| `kp_goal_relaxed` | `0.16` |
| `kd_goal` | `0.3` |
| `c_field` | `5.0` |
| `c_perp` | `8.0` |
| `r_l` | `2.5` |
| `r_la` | `1.0` |

#### `paper_geometric`

| Parameter | Value |
| --- | ---: |
| `kp_goal_relaxed` | `0.12` |
| `kp_geom` | `5.0` |
| `c_field` | `5.0` |
| `c_perp` | `8.0` |
| `speed_limit` | `0.8` |
| `r_l` | `2.5` |
| `r_la` | `1.0` |

#### `apf`, `haddadin`, `sabattini`

- no extra method-specific override in `METHOD_CONFIG_OVERRIDES`

### 3.5 Webots Crazyflie dynamic benchmark

Primary source:

- `scripts/run_webots_crazyflie_dynamic_benchmarks.py`

#### `paper_pd_3d`

| Parameter | Value |
| --- | ---: |
| `c_field` | `12.0` |
| `c_perp` | `16.0` |
| `speed_limit` | `0.34` |
| `kp_goal` | `0.08` |
| `kp_goal_relaxed` | `0.08` |
| `kd_goal` | `0.52` |
| `max_acceleration` | `2.0` |
| `max_speed_norm` | `0.50` |

#### `paper_geometric_3d`

| Parameter | Value |
| --- | ---: |
| `c_field` | `16.0` |
| `c_perp` | `18.0` |
| `speed_limit` | `0.34` |
| `kp_goal` | `0.10` |
| `kp_goal_relaxed` | `0.06` |
| `kd_goal` | `0.45` |
| `kp_geom` | `0.28` |
| `max_acceleration` | `2.2` |
| `max_speed_norm` | `0.52` |

#### `apf_3d`

| Parameter | Value |
| --- | ---: |
| `kp_goal` | `0.08` |
| `kd_goal` | `0.45` |
| `c_field` | `14.0` |
| `r_la` | `2.0` |
| `max_acceleration` | `2.2` |
| `max_speed_norm` | `0.42` |
| `speed_limit` | `0.32` |

#### `haddadin_3d` and `sabattini_3d`

| Parameter | Value |
| --- | ---: |
| `kp_goal` | `0.08` |
| `kd_goal` | `0.45` |
| `r_l` | `4.0` |
| `r_la` | `2.0` |
| `max_acceleration` | `2.2` |
| `max_speed_norm` | `0.42` |
| `speed_limit` | `0.34` |

## 4. Interpretation rules

To keep the paper claims clean, we should describe the tuning as follows:

- core MFI equations are shared across environments
- environment/model adapters may require actuation-level tuning
- Webots and differential-drive benchmark overrides are tracking/interface tuning, not new navigation laws

## 5. Update policy

Whenever a major benchmark or simulator-facing parameter changes, update:

1. this file
2. the relevant benchmark output folder
3. `docs/algorithm_and_model_changes.md` if the change affects interpretation
