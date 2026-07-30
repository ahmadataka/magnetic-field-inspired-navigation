# Paper Thread Experiment Report

Date: 2026-07-30

This note is a compact experiment-side deliverable for the manuscript/paper thread.

It is meant to satisfy the paper-agent request for:

- what scripts were run or added,
- exact parameter settings,
- exact random seeds,
- raw CSV paths,
- summary CSV/MD paths,
- figure paths,
- top-line findings,
- failed cases and likely reasons,
- whether the paper draft should update the main claim.

## 1. Scripts used

Most relevant scripts already used in the current repo state:

- `scripts/run_dynamic_multiseed_2d.py`
- `scripts/run_dynamic_beta_sweep_multiseed_2d.py`
- `scripts/run_benchmarks_dynamic_2d.py`
- `scripts/run_benchmarks_dynamic_diff_drive_2d.py`
- `scripts/run_benchmarks_dynamic_3d.py`
- `scripts/run_benchmarks_dynamic_quadrotor_3d.py`
- `scripts/run_webots_epuck_dynamic_benchmarks.py`
- `scripts/run_webots_crazyflie_dynamic_benchmarks.py`
- `scripts/summarize_dynamic_benchmarks.py`
- `scripts/make_dynamic_paper_figures.py`

Recent benchmark-side tuning changes were made in:

- `scripts/run_benchmarks_dynamic_diff_drive_2d.py`
- `scripts/run_benchmarks_dynamic_3d.py`

Frozen tuning and benchmark-side parameter notes are documented in:

- `docs/parameter_and_tuning_record.md`

## 2. Exact seeds

### 2.1 Multi-seed dynamic 2D benchmark

Source:

- `scripts/run_dynamic_multiseed_2d.py`

Seeds used:

- `[0, 1, 2, 3, 4]`

### 2.2 Multi-seed beta sweep

Source:

- `scripts/run_dynamic_beta_sweep_multiseed_2d.py`

Seeds used:

- `[0, 1, 2, 3, 4]`

### 2.3 Single-run benchmark families

The following benchmark families are deterministic benchmark runs, not random-seed Monte Carlo studies:

- `dynamic_double_integrator_2d`
- `dynamic_differential_drive_2d`
- `dynamic_double_integrator_3d`
- `dynamic_quadrotor_3d`
- `webots_epuck_dynamic`
- `webots_crazyflie_dynamic`

## 3. Exact parameter references

Use:

- `docs/parameter_and_tuning_record.md`

This is now the canonical frozen parameter record for:

- core simulation defaults,
- paper-style 2D and 3D MFI settings,
- dynamic diff-drive benchmark overrides,
- dynamic 3D double-integrator benchmark overrides,
- Webots e-puck dynamic overrides,
- Webots Crazyflie dynamic overrides.

Important fairness note:

- dynamic differential-drive tuning is documented, but it does not yet use a strict held-out validation split,
- 3D double-integrator tuning is benchmark-side only and keeps the core MFI equations unchanged.

## 4. Raw CSV and benchmark artifact paths

### 4.1 Multi-seed 2D dynamic benchmark

- raw: `artifacts/benchmarks/dynamic_multiseed_2d/dynamic_multiseed_raw.csv`
- summary csv: `artifacts/benchmarks/dynamic_multiseed_2d/dynamic_multiseed_summary.csv`
- summary md: `artifacts/benchmarks/dynamic_multiseed_2d/dynamic_multiseed_summary.md`
- plot: `artifacts/benchmarks/dynamic_multiseed_2d/dynamic_multiseed_summary.png`

### 4.2 Multi-seed beta sweep

- raw: `artifacts/benchmarks/dynamic_beta_sweep_multiseed_2d/dynamic_beta_sweep_multiseed_raw.csv`
- summary csv: `artifacts/benchmarks/dynamic_beta_sweep_multiseed_2d/dynamic_beta_sweep_multiseed_summary.csv`
- summary md: `artifacts/benchmarks/dynamic_beta_sweep_multiseed_2d/dynamic_beta_sweep_multiseed_summary.md`
- plot: `artifacts/benchmarks/dynamic_beta_sweep_multiseed_2d/dynamic_beta_sweep_multiseed.png`

### 4.3 Dynamic 2D double integrator

- metrics: `artifacts/benchmarks/dynamic_double_integrator_2d/benchmark_metrics_dynamic_2d.csv`
- figure: `artifacts/benchmarks/dynamic_double_integrator_2d/benchmark_comparison_dynamic_2d.png`
- html: `artifacts/benchmarks/dynamic_double_integrator_2d/benchmark_comparison_dynamic_2d.html`

### 4.4 Dynamic 2D differential drive

- metrics: `artifacts/benchmarks/dynamic_differential_drive_2d/benchmark_metrics_dynamic_diff_drive_2d.csv`
- figure: `artifacts/benchmarks/dynamic_differential_drive_2d/benchmark_comparison_dynamic_diff_drive_2d.png`
- html: `artifacts/benchmarks/dynamic_differential_drive_2d/benchmark_comparison_dynamic_diff_drive_2d.html`

### 4.5 Dynamic 3D double integrator

- metrics: `artifacts/benchmarks/dynamic_double_integrator_3d/benchmark_metrics_dynamic_3d.csv`
- figure: `artifacts/benchmarks/dynamic_double_integrator_3d/benchmark_comparison_dynamic_3d.png`
- html: `artifacts/benchmarks/dynamic_double_integrator_3d/benchmark_comparison_dynamic_3d.html`

### 4.6 Dynamic 3D quadrotor

- metrics: `artifacts/benchmarks/dynamic_quadrotor_3d/benchmark_metrics_dynamic_quadrotor_3d.csv`
- figure: `artifacts/benchmarks/dynamic_quadrotor_3d/benchmark_comparison_dynamic_quadrotor_3d.png`
- html: `artifacts/benchmarks/dynamic_quadrotor_3d/benchmark_comparison_dynamic_quadrotor_3d.html`

### 4.7 Webots e-puck dynamic

- metrics: `artifacts/benchmarks/webots_epuck_dynamic/benchmark_metrics_dynamic_webots_epuck.csv`
- figure: `artifacts/benchmarks/webots_epuck_dynamic/benchmark_comparison_dynamic_webots_epuck.png`

### 4.8 Webots Crazyflie dynamic

- metrics: `artifacts/benchmarks/webots_crazyflie_dynamic/benchmark_metrics_webots_crazyflie_dynamic.csv`
- figure: `artifacts/benchmarks/webots_crazyflie_dynamic/benchmark_comparison_webots_crazyflie_dynamic.png`

## 5. Summary and publication-facing paths

Use these first in the paper thread:

- `artifacts/summaries/dynamic_master/dynamic_master_summary.csv`
- `artifacts/summaries/dynamic_master/dynamic_master_per_scenario.csv`
- `docs/dynamic_benchmark_master_summary.md`
- `docs/dynamic_benchmark_master_summary.html`
- `docs/dynamic_paper_figures.md`

Curated publication-style figures:

- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_double_integrator_3d.png`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_double_integrator_3d.pdf`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_epuck.png`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_epuck.pdf`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_crazyflie.png`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_crazyflie.pdf`

## 6. Top-line findings

### 6.1 Theory-aligned 2D evidence is the strongest part

- Multi-seed 2D dynamic results now exist.
- Multi-seed beta sweep now exists.
- These are the strongest empirical support for the theoretical dynamic-avoidance interpretation.

### 6.2 2D double integrator remains the cleanest benchmark family

From the current master summary:

- `MFI-PD`: `5/6` success
- `MFI-Geometric`: `5/6` success
- both with zero collision rate and zero safety-violation rate

This is the best main-paper evidence.

### 6.3 Dynamic differential drive improved but remains a supporting/limitation case

Current retained dynamic diff-drive summary:

- `MFI-PD`: `3/6` success
- `MFI-Geometric`: `4/6` success in the older master summary, but the currently retained benchmark folder shows both MFI variants succeed on four of six scene types while still failing the hardest head-on and mixed scenes

Practical interpretation:

- MFI still works in several dynamic nonholonomic cases,
- but the hardest head-on and mixed-field cases remain challenging under nonholonomic tracking constraints.

This should be framed honestly in the paper.

### 6.4 Dynamic 3D double integrator is now much more usable

After the latest 3D benchmark-side tuning:

- `MFI-PD`: `3/5` success, collision rate `0.2`
- `MFI-Geometric`: `3/5` success, collision rate `0.2`

Important scenario-level interpretation:

- both MFI variants now succeed on:
  - `moving_sphere_crossing_3d`
  - `wandering_sphere_3d`
  - `moving_prism_gate_3d`
- both still fail on:
  - `head_on_sphere_3d`
- `mixed_static_dynamic_field_3d` is now collision-free for MFI, but still not goal-reaching within the benchmark horizon

This is good enough for supporting 3D validation, but not for a universal 3D claim.

### 6.5 Webots validation is supportive, not primary

- Webots e-puck is useful embodied 2D evidence.
- Webots Crazyflie is useful embodied 3D support.
- The paper should not overstate Webots as proof of real-world deployment.

## 7. Failed cases and likely reasons

### 7.1 Head-on dynamic cases

Weak cases:

- `head_on_circle`
- `head_on_sphere_3d`

Likely reason:

- the obstacle closing direction aligns directly with the goal direction,
- the theoretical speed-ratio and turning-authority limits become active,
- benchmark-side tuning alone is not always enough to recover a clean avoidance maneuver in these geometries.

### 7.2 Mixed static-dynamic clutter

Weak cases:

- `mixed_static_dynamic_field`
- `mixed_static_dynamic_field_3d`

Likely reason:

- multiple obstacle influences reduce available maneuvering room,
- the reactive field remains safe more often than it remains fully convergent,
- nonholonomic or bounded-actuation effects worsen this in embodied models.

### 7.3 Differential-drive embodied limitation

Likely reason:

- the same reactive field must be realized through heading-tracking and forward-speed constraints,
- so some failures are due to model/tracker limitations rather than the idealized field logic itself.

## 8. Claim guidance for the paper draft

### 8.1 Main claim can be strengthened, but carefully

Safe strengthened claim:

- the same core MFI controller shows strong dynamic-obstacle performance in theory-aligned 2D benchmarks,
- remains effective across repeated trials and beta sweeps,
- and transfers with meaningful success to representative 3D and embodied robot simulations.

### 8.2 What should still not be claimed

Do not claim:

- global convergence for arbitrary dynamic obstacles,
- guaranteed avoidance for arbitrarily fast or adversarial head-on obstacles,
- prediction-based dynamic obstacle avoidance,
- that Webots validation proves deployment readiness.

### 8.3 Suggested paper-ready summary paragraph

Across repeated 2D dynamic trials, the magnetic-field-inspired controller remained strong in the theory-aligned double-integrator setting, while multi-seed beta sweeps showed that performance degrades most clearly in head-on encounters as obstacle speed approaches the robot speed. Additional 3D and Webots results support transfer beyond the planar point-mass case, but these should be framed as supporting validation rather than universal dynamic-obstacle guarantees, especially in tightly constrained head-on and mixed-field scenarios.

## 9. Best immediate inputs for the paper-side thread

If the paper-side agent wants the highest-value inputs first, use:

1. `docs/dynamic_benchmark_master_summary.md`
2. `docs/parameter_and_tuning_record.md`
3. `docs/dynamic_paper_figures.md`
4. `artifacts/summaries/dynamic_master/dynamic_master_summary.csv`
5. `artifacts/summaries/dynamic_paper_figures/figure_dynamic_double_integrator_3d.pdf`
6. `artifacts/benchmarks/dynamic_multiseed_2d/dynamic_multiseed_summary.md`
7. `artifacts/benchmarks/dynamic_beta_sweep_multiseed_2d/dynamic_beta_sweep_multiseed_summary.md`
