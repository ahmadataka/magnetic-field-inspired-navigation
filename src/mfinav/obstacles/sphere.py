from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..utils.math2d import EPS, _norm


@dataclass
class SphereObstacle:
    center: np.ndarray
    radius: float

    def __post_init__(self) -> None:
        self.center = np.asarray(self.center, dtype=float)
        if self.center.shape != (3,):
            raise ValueError("SphereObstacle requires a 3D center vector.")

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        delta = position - self.center
        distance = _norm(delta)
        if distance < EPS:
            return np.array([self.radius, 0.0, 0.0], dtype=float)
        return delta * max(distance - self.radius, EPS) / distance

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        _ = num_samples, angular_window
        return [self.closest_vector(position)]

    def clearance(self, position: np.ndarray) -> float:
        return _norm(position - self.center) - self.radius

    def ray_intersection_distance(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        max_range: float,
    ) -> float | None:
        rel = position - self.center
        b = 2.0 * float(np.dot(direction, rel))
        c = float(np.dot(rel, rel) - self.radius**2)
        disc = b * b - 4.0 * c
        if disc < 0.0:
            return None
        sqrt_disc = disc**0.5
        candidates = [(-b - sqrt_disc) / 2.0, (-b + sqrt_disc) / 2.0]
        valid = [t for t in candidates if EPS <= t <= max_range]
        if not valid:
            return None
        return min(valid)

    def set_time(self, time_s: float) -> None:
        _ = time_s

    def snapshot(self) -> dict[str, object]:
        return {
            "kind": "sphere",
            "center": [float(self.center[0]), float(self.center[1]), float(self.center[2])],
            "radius": float(self.radius),
        }
