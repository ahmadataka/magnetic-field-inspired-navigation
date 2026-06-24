from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..obstacles.circle import CircleObstacle
from ..obstacles.polygon import PolygonObstacle
from ..obstacles.dynamic import DynamicObstacleCollection, MovingCircleObstacle, MovingPolygonObstacle


@dataclass
class DynamicBenchmarkScenario:
    name: str
    start: np.ndarray
    goal: np.ndarray
    obstacles: DynamicObstacleCollection
    description: str


def make_dynamic_scenarios_2d() -> list[DynamicBenchmarkScenario]:
    return [
        DynamicBenchmarkScenario(
            name="moving_circle_crossing",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([12.0, 0.0], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    MovingCircleObstacle(
                        initial_center=np.array([6.0, -3.2], dtype=float),
                        radius=1.05,
                        velocity=np.array([0.0, 0.65], dtype=float),
                    )
                ]
            ),
            description="A circular obstacle sweeps upward across the straight path between start and goal.",
        ),
        DynamicBenchmarkScenario(
            name="moving_convex_sweeper",
            start=np.array([0.0, -0.6], dtype=float),
            goal=np.array([13.0, 0.9], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    MovingPolygonObstacle(
                        initial_vertices=np.array(
                            [[5.1, -3.2], [7.3, -3.2], [7.3, -1.0], [5.1, -1.0]],
                            dtype=float,
                        ),
                        velocity=np.array([0.0, 0.6], dtype=float),
                    ),
                    MovingPolygonObstacle(
                        initial_vertices=np.array(
                            [[8.9, 1.4], [10.5, 1.1], [10.9, 2.4], [9.2, 2.7]],
                            dtype=float,
                        ),
                        velocity=np.array([-0.1, -0.22], dtype=float),
                    ),
                ]
            ),
            description="A convex rectangle sweeps through the path while a second convex polygon drifts diagonally downstream.",
        ),
        DynamicBenchmarkScenario(
            name="moving_u_shape",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([12.5, 0.0], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    MovingPolygonObstacle(
                        initial_vertices=np.array(
                            [
                                [4.0, -2.7],
                                [8.2, -2.7],
                                [8.2, 2.7],
                                [4.0, 2.7],
                                [4.0, 1.7],
                                [7.1, 1.7],
                                [7.1, -1.7],
                                [4.0, -1.7],
                            ],
                            dtype=float,
                        ),
                        velocity=np.array([0.0, -0.18], dtype=float),
                    )
                ]
            ),
            description="A non-convex U-shape whose mouth faces the robot while the whole obstacle drifts downward during the encounter.",
        ),
        DynamicBenchmarkScenario(
            name="head_on_circle",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([12.0, 0.0], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    MovingCircleObstacle(
                        initial_center=np.array([10.5, 0.0], dtype=float),
                        radius=1.0,
                        velocity=np.array([-0.75, 0.0], dtype=float),
                    )
                ]
            ),
            description="A circular obstacle moves directly toward the robot along the main line-of-sight to create a head-on reactive avoidance problem.",
        ),
        DynamicBenchmarkScenario(
            name="wandering_circle",
            start=np.array([0.0, -0.4], dtype=float),
            goal=np.array([13.0, 0.6], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    MovingCircleObstacle(
                        initial_center=np.array([6.7, 0.0], dtype=float),
                        radius=1.1,
                        velocity=np.array([0.05, 0.12], dtype=float),
                        oscillation_amplitude=np.array([0.9, 1.1], dtype=float),
                        oscillation_frequency=np.array([0.8, 1.35], dtype=float),
                        oscillation_phase=np.array([0.4, 1.2], dtype=float),
                    )
                ]
            ),
            description="A dynamic circle follows a deterministic wandering motion that looks mildly random from the robot perspective.",
        ),
        DynamicBenchmarkScenario(
            name="mixed_static_dynamic_field",
            start=np.array([0.0, -0.2], dtype=float),
            goal=np.array([15.0, 0.3], dtype=float),
            obstacles=DynamicObstacleCollection(
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [[3.3, -2.8], [5.2, -2.8], [5.2, -0.7], [3.3, -0.7]],
                            dtype=float,
                        )
                    ),
                    MovingCircleObstacle(
                        initial_center=np.array([7.4, -2.6], dtype=float),
                        radius=0.95,
                        velocity=np.array([0.0, 0.58], dtype=float),
                    ),
                    MovingPolygonObstacle(
                        initial_vertices=np.array(
                            [
                                [9.6, -1.8],
                                [12.5, -1.8],
                                [12.5, 2.1],
                                [9.6, 2.1],
                                [9.6, 1.1],
                                [11.5, 1.1],
                                [11.5, -0.9],
                                [9.6, -0.9],
                            ],
                            dtype=float,
                        ),
                        velocity=np.array([0.0, -0.14], dtype=float),
                    ),
                    CircleObstacle(
                        center=np.array([13.2, 1.3], dtype=float),
                        radius=0.85,
                    ),
                ]
            ),
            description="A mixed field combining static convex obstacles, a moving circle crossing the route, and a drifting non-convex polygon downstream.",
        ),
    ]
