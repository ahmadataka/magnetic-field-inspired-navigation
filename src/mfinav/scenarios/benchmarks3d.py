from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..obstacles.collection import ObstacleCollection
from ..obstacles.sphere import SphereObstacle


@dataclass
class BenchmarkScenario3D:
    name: str
    start: np.ndarray
    goal: np.ndarray
    obstacles: ObstacleCollection
    description: str


def make_default_scenarios_3d() -> list[BenchmarkScenario3D]:
    return [
        BenchmarkScenario3D(
            name="single_sphere",
            start=np.array([0.0, 0.0, 0.0], dtype=float),
            goal=np.array([10.0, 1.5, 1.0], dtype=float),
            obstacles=ObstacleCollection(obstacles=[SphereObstacle(center=np.array([5.0, 0.5, 0.5]), radius=1.4)]),
            description="Single sphere between start and goal.",
        ),
        BenchmarkScenario3D(
            name="double_sphere_gap",
            start=np.array([0.0, 0.0, 0.0], dtype=float),
            goal=np.array([12.5, 0.0, 0.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    SphereObstacle(center=np.array([5.2, -1.6, 0.0]), radius=1.6),
                    SphereObstacle(center=np.array([5.8, 1.6, 0.0]), radius=1.6),
                ]
            ),
            description="Two spheres with a narrow central passage.",
        ),
        BenchmarkScenario3D(
            name="sphere_cluster",
            start=np.array([0.0, -0.5, 0.0], dtype=float),
            goal=np.array([14.0, 1.0, 1.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    SphereObstacle(center=np.array([4.8, -1.2, 0.2]), radius=1.4),
                    SphereObstacle(center=np.array([7.2, 1.0, 0.8]), radius=1.6),
                    SphereObstacle(center=np.array([9.8, -0.6, 1.4]), radius=1.4),
                    SphereObstacle(center=np.array([11.5, 1.6, 0.1]), radius=1.3),
                ]
            ),
            description="Cluster of multiple spheres requiring repeated avoidance in 3D.",
        ),
        BenchmarkScenario3D(
            name="offset_cavity",
            start=np.array([0.0, 0.0, 0.0], dtype=float),
            goal=np.array([13.5, 0.0, 0.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    SphereObstacle(center=np.array([6.0, -2.2, 0.0]), radius=1.4),
                    SphereObstacle(center=np.array([6.0, 2.2, 0.0]), radius=1.4),
                    SphereObstacle(center=np.array([9.3, 0.8, 0.0]), radius=1.5),
                    SphereObstacle(center=np.array([8.0, 0.0, 2.4]), radius=1.2),
                ]
            ),
            description="A reachable non-convex-like cavity composed from offset spheres.",
        ),
        BenchmarkScenario3D(
            name="archway",
            start=np.array([0.0, 0.0, 0.0], dtype=float),
            goal=np.array([13.0, 0.0, 0.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    SphereObstacle(center=np.array([6.5, -2.0, 0.0]), radius=1.5),
                    SphereObstacle(center=np.array([6.5, 2.0, 0.0]), radius=1.5),
                    SphereObstacle(center=np.array([8.5, 0.0, 2.4]), radius=1.6),
                    SphereObstacle(center=np.array([8.5, 0.0, -2.4]), radius=1.6),
                ]
            ),
            description="A non-convex archway assembled from spheres that requires 3D routing.",
        ),
    ]


def make_stress_scenarios_3d() -> list[BenchmarkScenario3D]:
    return [
        BenchmarkScenario3D(
            name="wide_cavity_trap",
            start=np.array([0.0, 0.0, 0.0], dtype=float),
            goal=np.array([13.5, 0.0, 0.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    SphereObstacle(center=np.array([6.0, -2.8, 0.0]), radius=1.4),
                    SphereObstacle(center=np.array([6.0, 2.8, 0.0]), radius=1.4),
                    SphereObstacle(center=np.array([8.6, 0.0, -2.8]), radius=1.4),
                    SphereObstacle(center=np.array([8.6, 0.0, 2.8]), radius=1.4),
                    SphereObstacle(center=np.array([9.5, 0.0, 0.0]), radius=1.3),
                ]
            ),
            description="A deeper cavity stress case that still traps the current purely local 3D controller.",
        ),
    ]
