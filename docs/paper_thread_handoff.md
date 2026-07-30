# Paper Thread Handoff

Use this note as the starting context for the Codex thread that will work on the paper/manuscript side.

For the newest compact experiment-side deliverable, also read:

- `docs/paper_thread_experiment_report.md`

## Repo

- repository: `magnetic-field-inspired-navigation`
- local path: `/Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation`
- primary branch already pushed: `main`

## Main purpose of this repo

This repo is the clean Python implementation and benchmarking environment for magnetic-field-inspired navigation (MFI), including:

- 2D double-integrator
- 3D double-integrator
- 2D differential-drive
- 3D quadrotor model
- Webots e-puck validation
- Webots Crazyflie validation
- static-obstacle benchmarks
- dynamic-obstacle benchmarks
- comparison baselines such as `APF`, `Haddadin`, and `Sabattini`

## Most relevant output folders for the paper

### 1. Master dynamic benchmark summary

Use these first if the paper thread needs tables or overall benchmark conclusions:

- `artifacts/summaries/dynamic_master/dynamic_master_summary.csv`
- `artifacts/summaries/dynamic_master/dynamic_master_per_scenario.csv`
- `docs/dynamic_benchmark_master_summary.md`
- `docs/dynamic_benchmark_master_summary.html`

These consolidate the currently available dynamic benchmark results across:

- `dynamic_double_integrator_2d`
- `dynamic_differential_drive_2d`
- `dynamic_double_integrator_3d`
- `dynamic_quadrotor_3d`
- `webots_epuck_dynamic`
- `webots_crazyflie_dynamic`

### 2. Curated publication-style trajectory figures

Use these for manuscript figures first, before using the larger benchmark sheets:

- `docs/dynamic_paper_figures.md`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_double_integrator_3d.png`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_double_integrator_3d.pdf`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_epuck.png`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_epuck.pdf`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_crazyflie.png`
- `artifacts/summaries/dynamic_paper_figures/figure_dynamic_webots_crazyflie.pdf`

These are cleaner than the full benchmark plots because they:

- show only `MFI-PD`, `MFI-Geometric`, and one baseline
- use obstacle snapshots plus motion arrows
- use 2D projections for 3D cases

### 3. Dynamic beta sweep

Use this if the paper discusses sensitivity to obstacle motion or tuning:

- `artifacts/benchmarks/dynamic_beta_sweep_2d/dynamic_beta_sweep.png`
- `artifacts/benchmarks/dynamic_beta_sweep_2d/dynamic_beta_sweep_raw.csv`
- `artifacts/benchmarks/dynamic_beta_sweep_2d/dynamic_beta_sweep_summary.csv`
- `artifacts/benchmarks/dynamic_beta_sweep_2d/dynamic_beta_sweep_summary.md`

### 4. Static and dynamic benchmark sheets

Use these as supporting material, appendices, or for verifying a claim:

- `artifacts/benchmarks/dynamic_double_integrator_2d/`
- `artifacts/benchmarks/dynamic_differential_drive_2d/`
- `artifacts/benchmarks/dynamic_double_integrator_3d/`
- `artifacts/benchmarks/dynamic_quadrotor_3d/`
- `artifacts/benchmarks/webots_epuck_dynamic/`
- `artifacts/benchmarks/webots_crazyflie_dynamic/`
- `artifacts/benchmarks/double_integrator_2d/`
- `artifacts/benchmarks/double_integrator_3d/`
- `artifacts/benchmarks/differential_drive_2d/`
- `artifacts/benchmarks/quadrotor_3d/`
- `artifacts/benchmarks/webots_epuck_static/`
- `artifacts/benchmarks/webots_crazyflie_static/`

## Most relevant scripts

If the paper thread needs to regenerate or verify outputs, start from these:

- `scripts/summarize_dynamic_benchmarks.py`
- `scripts/make_dynamic_paper_figures.py`
- `scripts/run_dynamic_beta_sweep_2d.py`
- `scripts/run_benchmarks_dynamic_2d.py`
- `scripts/run_benchmarks_dynamic_diff_drive_2d.py`
- `scripts/run_benchmarks_dynamic_3d.py`
- `scripts/run_benchmarks_dynamic_quadrotor_3d.py`
- `scripts/run_webots_epuck_dynamic_benchmarks.py`
- `scripts/run_webots_crazyflie_dynamic_benchmarks.py`

## Relevant implementation files

If the paper thread needs to inspect the actual algorithm/model code:

- `src/mfinav/navigators/`
- `src/mfinav/models/`
- `src/mfinav/obstacles/`
- `src/mfinav/scenarios/`
- `src/mfinav/sim/runner.py`
- `src/mfinav/sim/differential_drive.py`
- `src/mfinav/sim/quadrotor.py`

## Important modeling/algorithm notes

### MFI variants

The repo distinguishes at least:

- `paper_pd` / `paper_pd_3d`
- `paper_geometric` / `paper_geometric_3d`

These are the most important MFI variants for the paper.

### Baselines

The main comparison baselines currently implemented are:

- `APF`
- `Haddadin`
- `Sabattini`

### Haddadin caveat

The current `Haddadin` implementation is less purely reactive than MFI because it uses obstacle-center information. That is already a meaningful discussion point for the paper: MFI performs competitively while relying on less global obstacle information.

### Visualization caveat

Do not use the large all-method benchmark sheets as the main paper figures unless necessary. They are good for repo validation, but the curated figures in `artifacts/summaries/dynamic_paper_figures/` are more publication-friendly.

### Crazyflie curated figure caveat

The curated Crazyflie paper figure intentionally uses the validated `moving_sphere_crossing_3d` case, not the harder prism-gate case, because one baseline could blow up and make the panel unreadable. This was a figure-readability choice, not a change to the algorithm itself.

## Recommended order for the paper thread

If the manuscript-side agent needs to work efficiently, use this order:

1. Read `docs/dynamic_benchmark_master_summary.md`
2. Read `docs/dynamic_paper_figures.md`
3. Inspect `artifacts/summaries/dynamic_master/dynamic_master_summary.csv`
4. Inspect the curated figure files in `artifacts/summaries/dynamic_paper_figures/`
5. Only then inspect the larger benchmark folders if needed

## Suggested prompt for the paper-side agent

You can give the paper thread something close to this:

> Use `/Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation/docs/paper_thread_handoff.md` as the repo guide. Prioritize `docs/dynamic_benchmark_master_summary.md`, `docs/dynamic_paper_figures.md`, `artifacts/summaries/dynamic_master/`, and `artifacts/summaries/dynamic_paper_figures/`. Use the curated figures as the first candidates for the paper. Only inspect the full benchmark folders if you need backup evidence, additional cases, or raw comparison details.
