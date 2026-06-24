from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..obstacles.dynamic import DynamicObstacleCollection, MovingPrismObstacle, MovingSphereObstacle
from ..obstacles.prism import PrismObstacle
from ..obstacles.sphere import SphereObstacle


@dataclass
class DynamicBenchmarkScenario3D:
    name: str
    start: np.ndarray
    goal: np.ndarray
    obstacles: DynamicObstacleCollection
    description: str


def make_dynamic_scenarios_3d() -> list[DynamicBenchmarkScenario3D]:
    return [
        DynamicBenchmarkScenario3D(
            name="moving_sphere_crossing_3d",
            start=np.array([0.0, 0.0, 0.0], dtype=float),
            goal=np.array([11.0, 0.5, 0.6], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    MovingSphereObstacle(
                        initial_center=np.array([5.8, -3.0, 0.3], dtype=float),
                        radius=1.25,
                        velocity=np.array([0.0, 0.65, 0.0], dtype=float),
                    )
                ]
            ),
            description="A moving sphere crosses the direct route in 3D.",
        ),
        DynamicBenchmarkScenario3D(
            name="head_on_sphere_3d",
            start=np.array([0.0, 0.0, 0.0], dtype=float),
            goal=np.array([11.5, 0.0, 0.0], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    MovingSphereObstacle(
                        initial_center=np.array([10.2, 0.0, 0.0], dtype=float),
                        radius=1.1,
                        velocity=np.array([-0.8, 0.0, 0.0], dtype=float),
                    )
                ]
            ),
            description="A head-on moving sphere approaches directly along the goal line.",
        ),
        DynamicBenchmarkScenario3D(
            name="wandering_sphere_3d",
            start=np.array([0.0, -0.4, 0.0], dtype=float),
            goal=np.array([12.0, 1.0, 1.0], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    MovingSphereObstacle(
                        initial_center=np.array([6.4, 0.0, 0.6], dtype=float),
                        radius=1.2,
                        velocity=np.array([0.05, 0.10, 0.03], dtype=float),
                        oscillation_amplitude=np.array([0.9, 1.1, 0.7], dtype=float),
                        oscillation_frequency=np.array([0.7, 1.15, 0.95], dtype=float),
                        oscillation_phase=np.array([0.2, 1.1, 2.0], dtype=float),
                    )
                ]
            ),
            description="A deterministic wandering sphere introduces random-looking 3D obstacle motion.",
        ),
        DynamicBenchmarkScenario3D(
            name="moving_prism_gate_3d",
            start=np.array([0.0, -0.5, 0.0], dtype=float),
            goal=np.array([12.0, 1.4, 0.6], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    MovingPrismObstacle(
                        initial_vertices_xy=np.array(
                            [[5.0, -2.6], [7.8, -2.6], [7.8, -0.6], [5.0, -0.6]],
                            dtype=float,
                        ),
                        z_min=-1.1,
                        z_max=1.1,
                        velocity=np.array([0.0, 0.48, 0.0], dtype=float),
                    ),
                    MovingSphereObstacle(
                        initial_center=np.array([9.6, 1.9, 0.8], dtype=float),
                        radius=1.0,
                        velocity=np.array([-0.1, -0.16, 0.0], dtype=float),
                    ),
                ]
            ),
            description="A moving convex prism sweeps through the route while a second obstacle drifts downstream.",
        ),
        DynamicBenchmarkScenario3D(
            name="mixed_static_dynamic_field_3d",
            start=np.array([0.0, 0.0, 0.0], dtype=float),
            goal=np.array([13.5, 0.5, 0.8], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    PrismObstacle(
                        vertices_xy=np.array(
                            [[3.6, -2.2], [5.4, -2.2], [5.4, -0.5], [3.6, -0.5]],
                            dtype=float,
                        ),
                        z_min=-1.0,
                        z_max=1.0,
                    ),
                    MovingSphereObstacle(
                        initial_center=np.array([7.0, -2.5, 0.3], dtype=float),
                        radius=1.0,
                        velocity=np.array([0.0, 0.55, 0.0], dtype=float),
                    ),
                    MovingPrismObstacle(
                        initial_vertices_xy=np.array(
                            [
                                [9.2, -1.8],
                                [12.0, -1.8],
                                [12.0, 2.1],
                                [9.2, 2.1],
                                [9.2, 0.9],
                                [11.0, 0.9],
                                [11.0, -0.7],
                                [9.2, -0.7],
                            ],
                            dtype=float,
                        ),
                        z_min=-1.2,
                        z_max=1.2,
                        velocity=np.array([0.0, -0.12, 0.0], dtype=float),
                    ),
                    SphereObstacle(center=np.array([12.8, 1.5, 1.0], dtype=float), radius=0.9),
                ]
            ),
            description="A mixed static-plus-dynamic 3D field with convex and non-convex structure.",
        ),
    ]
