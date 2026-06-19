from __future__ import annotations

from .config.simulation import (
    SimulationConfig,
    make_paper_faithful_config,
    make_paper_geometric_config,
    make_paper_pd_config,
    make_paper_pd_3d_config,
    make_pragmatic_config,
)
from .metrics.benchmark import compute_metrics
from .models.double_integrator import DoubleIntegratorModel, DoubleIntegratorState
from .navigators.apf import ArtificialPotentialFieldNavigator
from .navigators.mfi3d import MagneticFieldNavigator3D
from .navigators.mfi import (
    BoundaryFollowingField,
    CollisionAvoidanceField,
    GoalRelaxationController,
    MagneticFieldNavigator,
    ReferenceNavigator,
)
from .obstacles.base import Obstacle
from .obstacles.circle import CircleObstacle
from .obstacles.collection import ObstacleCollection
from .obstacles.polygon import PolygonObstacle
from .obstacles.sphere import SphereObstacle
from .scenarios.benchmarks import BenchmarkScenario, make_default_scenarios
from .scenarios.benchmarks3d import BenchmarkScenario3D, make_default_scenarios_3d, make_stress_scenarios_3d
from .sensing.local import LocalSensingModel, LocalSensingObservation
from .sim.runner import simulate, write_history_csv
from .utils.math2d import (
    EPS,
    _closest_point_on_segment,
    _cross2,
    _embed_2d,
    _norm,
    _perp_left,
    _project_2d,
    _signed_angle,
    _skew3,
    _surface_current_from_observation,
    _unit,
)

__all__ = [
    "ArtificialPotentialFieldNavigator",
    "BenchmarkScenario",
    "BoundaryFollowingField",
    "CircleObstacle",
    "CollisionAvoidanceField",
    "compute_metrics",
    "DoubleIntegratorModel",
    "DoubleIntegratorState",
    "EPS",
    "GoalRelaxationController",
    "LocalSensingModel",
    "LocalSensingObservation",
    "MagneticFieldNavigator",
    "MagneticFieldNavigator3D",
    "Obstacle",
    "ObstacleCollection",
    "PolygonObstacle",
    "SphereObstacle",
    "ReferenceNavigator",
    "SimulationConfig",
    "BenchmarkScenario3D",
    "_closest_point_on_segment",
    "_cross2",
    "_embed_2d",
    "_norm",
    "_perp_left",
    "_project_2d",
    "_signed_angle",
    "_skew3",
    "_surface_current_from_observation",
    "_unit",
    "make_default_scenarios",
    "make_default_scenarios_3d",
    "make_stress_scenarios_3d",
    "make_paper_faithful_config",
    "make_paper_geometric_config",
    "make_paper_pd_config",
    "make_paper_pd_3d_config",
    "make_pragmatic_config",
    "simulate",
    "write_history_csv",
]
