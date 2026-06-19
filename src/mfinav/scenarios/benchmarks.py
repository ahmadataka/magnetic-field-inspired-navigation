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
            name="slalom_multi_convex",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([14.5, 0.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [[3.0, -3.0], [4.6, -3.0], [4.6, -0.5], [3.0, -0.5]],
                            dtype=float,
                        )
                    ),
                    PolygonObstacle(
                        vertices=np.array(
                            [[5.8, 0.5], [7.4, 0.5], [7.4, 3.0], [5.8, 3.0]],
                            dtype=float,
                        )
                    ),
                    PolygonObstacle(
                        vertices=np.array(
                            [[8.6, -3.0], [10.2, -3.0], [10.2, -0.5], [8.6, -0.5]],
                            dtype=float,
                        )
                    ),
                    PolygonObstacle(
                        vertices=np.array(
                            [[11.4, 0.5], [13.0, 0.5], [13.0, 3.0], [11.4, 3.0]],
                            dtype=float,
                        )
                    ),
                ]
            ),
            description="Alternating convex obstacles forming a slalom-like corridor.",
        ),
        BenchmarkScenario(
            name="facing_u_shape",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([10.8, 0.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [
                                [4.0, -2.5],
                                [8.0, -2.5],
                                [8.0, 2.5],
                                [4.0, 2.5],
                                [4.0, 1.5],
                                [7.0, 1.5],
                                [7.0, -1.5],
                                [4.0, -1.5],
                            ],
                            dtype=float,
                        )
                    )
                ]
            ),
            description="Non-convex U-shaped obstacle whose concavity faces the approaching robot.",
        ),
        BenchmarkScenario(
            name="mixed_concave_field",
            start=np.array([0.0, -0.4], dtype=float),
            goal=np.array([15.0, 0.5], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [
                                [3.2, -3.0],
                                [7.4, -3.0],
                                [7.4, 3.0],
                                [3.2, 3.0],
                                [3.2, 1.9],
                                [6.2, 1.9],
                                [6.2, 0.7],
                                [3.2, 0.7],
                                [3.2, -0.7],
                                [6.2, -0.7],
                                [6.2, -1.9],
                                [3.2, -1.9],
                            ],
                            dtype=float,
                        )
                    ),
                    PolygonObstacle(
                        vertices=np.array(
                            [[9.4, -2.3], [11.0, -2.3], [11.0, -0.4], [9.4, -0.4]],
                            dtype=float,
                        )
                    ),
                    PolygonObstacle(
                        vertices=np.array(
                            [[11.8, 0.3], [13.6, 0.0], [14.0, 1.6], [12.3, 1.9]],
                            dtype=float,
                        )
                    ),
                ]
            ),
            description="A true concave-facing obstacle followed by additional downstream obstacles.",
        ),
    ]
