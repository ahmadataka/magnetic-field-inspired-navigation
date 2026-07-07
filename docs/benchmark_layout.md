# Benchmark Layout

Benchmark outputs are organized by purpose instead of being written directly into `artifacts/`.

## Artifacts

- `artifacts/benchmarks/<suite>/`
  - Main benchmark outputs for each model/scenario family.
  - Typical files: comparison plot, interactive HTML, metrics CSV.
- `artifacts/smoke_tests/<name>/`
  - Quick Webots validation runs and controller smoke-test outputs.
- `artifacts/diagnostics/<name>/`
  - Focused debugging runs such as the non-convex diagnostic.
- `artifacts/reference/<name>/`
  - Reference trajectories and simple baseline outputs.
- `artifacts/summaries/<name>/`
  - Cross-benchmark summary tables such as the paper-style aggregate metrics.

## Webots Generated Worlds

Generated Webots benchmark worlds are grouped under:

- `webots/worlds/generated/epuck/static/`
- `webots/worlds/generated/epuck/dynamic/`
- `webots/worlds/generated/crazyflie/static/`
- `webots/worlds/generated/crazyflie/3d/`

The benchmark scripts recreate these generated directories automatically.
