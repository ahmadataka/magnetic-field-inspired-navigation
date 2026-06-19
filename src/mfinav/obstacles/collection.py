from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .base import Obstacle
from ..utils.math2d import _norm


@dataclass
class ObstacleCollection:
    obstacles: list[Obstacle]

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        if not self.obstacles:
            return np.array([1e6, 0.0], dtype=float)
        closest = self.obstacles[0].closest_vector(position)
        min_distance = _norm(closest)
        for obstacle in self.obstacles[1:]:
            candidate = obstacle.closest_vector(position)
            distance = _norm(candidate)
            if distance < min_distance:
                closest = candidate
                min_distance = distance
        return closest

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        vectors: list[np.ndarray] = []
        for obstacle in self.obstacles:
            vectors.extend(obstacle.sensed_vectors(position, num_samples, angular_window))
        return vectors

    def raycast_vectors(self, position: np.ndarray, num_samples: int, max_range: float) -> list[np.ndarray]:
        vectors: list[np.ndarray] = []
        angles = np.linspace(-math.pi, math.pi, max(8, num_samples), endpoint=False)
        for angle in angles:
            direction = np.array([math.cos(angle), math.sin(angle)], dtype=float)
            best_t: float | None = None
            for obstacle in self.obstacles:
                candidate = obstacle.ray_intersection_distance(position, direction, max_range)
                if candidate is not None and (best_t is None or candidate < best_t):
                    best_t = candidate
            if best_t is not None:
                vectors.append(direction * best_t)
        return vectors

    def clearance(self, position: np.ndarray) -> float:
        if not self.obstacles:
            return math.inf
        return min(obstacle.clearance(position) for obstacle in self.obstacles)
