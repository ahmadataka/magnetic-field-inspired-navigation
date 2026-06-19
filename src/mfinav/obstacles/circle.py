from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from ..utils.math2d import EPS, _norm


@dataclass
class CircleObstacle:
    center: np.ndarray
    radius: float

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        delta = position - self.center
        distance = _norm(delta)
        if distance < EPS:
            return np.array([self.radius, 0.0], dtype=float)
        return delta * max(distance - self.radius, EPS) / distance

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        delta = position - self.center
        base_angle = math.atan2(delta[1], delta[0])
        samples: list[np.ndarray] = []
        if num_samples <= 1:
            angles = [base_angle]
        else:
            angles = np.linspace(base_angle - angular_window, base_angle + angular_window, num_samples)
        for angle in angles:
            surface_point = self.center + self.radius * np.array([math.cos(angle), math.sin(angle)], dtype=float)
            samples.append(surface_point - position)
        return samples

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
        sqrt_disc = math.sqrt(disc)
        candidates = [(-b - sqrt_disc) / 2.0, (-b + sqrt_disc) / 2.0]
        valid = [t for t in candidates if EPS <= t <= max_range]
        if not valid:
            return None
        return min(valid)
