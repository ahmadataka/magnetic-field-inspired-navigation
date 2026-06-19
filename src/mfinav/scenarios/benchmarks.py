from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..obstacles.collection import ObstacleCollection
from ..obstacles.polygon import PolygonObstacle


@dataclass
class BenchmarkScenario:
    name: str
    start: np.ndarray
    goal: np.ndarray
    obstacles: ObstacleCollection
    description: str


def make_default_scenarios() -> list[BenchmarkScenario]:
    return [
        BenchmarkScenario(
            name="single_convex",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([10.0, 1.5], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [[4.0, -1.0], [6.0, -1.0], [6.0, 1.0], [4.0, 1.0]],
                            dtype=float,
                        )
                    )
                ]
            ),
            description="Single convex rectangular obstacle between start and goal.",
        ),
        BenchmarkScenario(
            name="convex_cluster",
            start=np.array([0.0, -1.0], dtype=float),
            goal=np.array([11.0, 1.5], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [[3.2, -1.0], [4.8, -1.0], [4.8, 0.8], [3.2, 0.8]],
                            dtype=float,
                        )
                    ),
                    PolygonObstacle(
                        vertices=np.array(
                            [[5.6, 0.4], [7.0, 0.1], [7.6, 1.5], [6.2, 1.8]],
                            dtype=float,
                        )
                    ),
                    PolygonObstacle(
                        vertices=np.array(
                            [[7.1, -1.7], [8.3, -1.1], [7.9, 0.0], [6.8, -0.4]],
                            dtype=float,
                        )
                    ),
                ]
            ),
            description="Several separated convex polygonal obstacles requiring repeated boundary following.",
        ),
        BenchmarkScenario(
            name="nonconvex_u_shape",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([10.5, 0.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [
                                [4.0, -2.5],
                                [8.0, -2.5],
                                [8.0, -1.2],
                                [5.4, -1.2],
                                [5.4, 1.2],
                                [8.0, 1.2],
                                [8.0, 2.5],
                                [4.0, 2.5],
                            ],
                            dtype=float,
                        )
                    )
                ]
            ),
            description="Non-convex U-shaped polygon obstacle with a right-facing cavity.",
        ),
    ]
