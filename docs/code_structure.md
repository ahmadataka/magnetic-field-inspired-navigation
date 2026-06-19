# Code Structure

This repository is organized so that robot models, navigation algorithms, and
simulation infrastructure are separate.

## Package Layout

- `src/mfinav/models/`
  Robot dynamics and state propagation.
  Current implementation:
  - `DoubleIntegratorState`
  - `DoubleIntegratorModel`

- `src/mfinav/navigators/`
  Navigation and control logic independent of the robot model.
  Current implementations:
  - `MagneticFieldNavigator`
  - `ArtificialPotentialFieldNavigator`

- `src/mfinav/obstacles/`
  Obstacle geometry and geometric queries.
  Current implementations:
  - `CircleObstacle`
  - `PolygonObstacle`
  - `ObstacleCollection`

- `src/mfinav/sensing/`
  Local sensing assumptions and observation types.
  Current implementation:
  - `LocalSensingModel`
  - `LocalSensingObservation`

- `src/mfinav/scenarios/`
  Benchmark scenario definitions.
  Current implementation:
  - `BenchmarkScenario`
  - `make_default_scenarios`

- `src/mfinav/sim/`
  Generic simulation runner and history export.
  Current implementation:
  - `simulate`
  - `write_history_csv`

- `src/mfinav/metrics/`
  Benchmark metrics and evaluation helpers.
  Current implementation:
  - `compute_metrics`

- `src/mfinav/config/`
  Shared simulation and controller configuration.
  Current implementation:
  - `SimulationConfig`
  - paper/pragmatic config presets

- `src/mfinav/utils/`
  Reusable low-level helpers such as 2D vector math.

## Design Intent

The main dependency direction is:

`obstacles + sensing + navigator + model -> simulator -> scripts`

That means:

- robot models can be added without rewriting the navigation algorithms
- navigation algorithms can be added without rewriting the robot model
- the same scenarios and metrics can be reused for comparisons

## Current Compatibility

The legacy import surface is intentionally preserved:

- `mfinav.__init__` still exports the high-level symbols used by the scripts
- `mfinav.double_integrator` is now a compatibility layer that re-exports the
  modular implementation

This keeps the current scripts working while allowing future work to use the
newer modular modules directly.

## Adding A New Robot Model

To add a new robot model later:

1. add a state type and model class under `src/mfinav/models/`
2. implement the propagation step
3. connect that model to the simulator
4. adapt command interpretation if the robot uses a different control input

## Adding A New Navigation Method

To add another navigation algorithm later:

1. add a new navigator under `src/mfinav/navigators/`
2. implement `command(state, goal, obstacle)`
3. reuse the same sensing, scenarios, simulator, and metrics

