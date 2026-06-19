from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from ..utils.math2d import EPS, _closest_point_on_segment, _norm


@dataclass
class PrismObstacle:
    vertices_xy: np.ndarray
    z_min: float
    z_max: float

    def __post_init__(self) -> None:
        self.vertices_xy = np.asarray(self.vertices_xy, dtype=float)
        if self.vertices_xy.ndim != 2 or self.vertices_xy.shape[1] != 2 or len(self.vertices_xy) < 3:
            raise ValueError("PrismObstacle requires an array of shape (N, 2) with N >= 3.")
        if self.z_max <= self.z_min:
            raise ValueError("PrismObstacle requires z_max > z_min.")

    def _edges(self) -> list[tuple[np.ndarray, np.ndarray]]:
        return [
            (self.vertices_xy[i], self.vertices_xy[(i + 1) % len(self.vertices_xy)])
            for i in range(len(self.vertices_xy))
        ]

    def _contains_xy(self, position_xy: np.ndarray) -> bool:
        x = float(position_xy[0])
        y = float(position_xy[1])
        inside = False
        for start, end in self._edges():
            x1, y1 = float(start[0]), float(start[1])
            x2, y2 = float(end[0]), float(end[1])
            if abs(y2 - y1) < EPS:
                continue
            intersects = (y1 > y) != (y2 > y)
            if not intersects:
                continue
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x_cross >= x:
                inside = not inside
        return inside

    def _closest_xy_boundary_point(self, position_xy: np.ndarray) -> tuple[np.ndarray, float]:
        best_point: np.ndarray | None = None
        best_distance = math.inf
        for start, end in self._edges():
            point, _ = _closest_point_on_segment(position_xy, start, end)
            distance = _norm(point - position_xy)
            if distance < best_distance:
                best_point = point
                best_distance = distance
        if best_point is None:
            return np.array([1e6, 0.0], dtype=float), math.inf
        return best_point, best_distance

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        position = np.asarray(position, dtype=float)
        if position.shape != (3,):
            raise ValueError("PrismObstacle requires a 3D position vector.")

        position_xy = position[:2]
        position_z = float(position[2])
        clamped_z = min(max(position_z, self.z_min), self.z_max)
        boundary_xy, _ = self._closest_xy_boundary_point(position_xy)
        side_point = np.array([boundary_xy[0], boundary_xy[1], clamped_z], dtype=float)
        candidates = [side_point]

        if self._contains_xy(position_xy):
            candidates.append(np.array([position_xy[0], position_xy[1], self.z_min], dtype=float))
            candidates.append(np.array([position_xy[0], position_xy[1], self.z_max], dtype=float))

        best_vector = candidates[0] - position
        best_distance = _norm(best_vector)
        for candidate in candidates[1:]:
            vector = candidate - position
            distance = _norm(vector)
            if distance < best_distance:
                best_vector = vector
                best_distance = distance
        return best_vector

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        _ = num_samples, angular_window
        return [self.closest_vector(position)]

    def clearance(self, position: np.ndarray) -> float:
        position = np.asarray(position, dtype=float)
        position_xy = position[:2]
        position_z = float(position[2])
        inside_xy = self._contains_xy(position_xy)
        within_z = self.z_min <= position_z <= self.z_max
        _, side_distance = self._closest_xy_boundary_point(position_xy)
        if inside_xy and within_z:
            return -min(side_distance, position_z - self.z_min, self.z_max - position_z)
        return _norm(self.closest_vector(position))

