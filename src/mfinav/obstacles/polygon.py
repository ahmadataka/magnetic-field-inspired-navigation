from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from ..utils.math2d import EPS, _closest_point_on_segment, _cross2, _norm


@dataclass
class PolygonObstacle:
    vertices: np.ndarray

    def __post_init__(self) -> None:
        self.vertices = np.asarray(self.vertices, dtype=float)
        if self.vertices.ndim != 2 or self.vertices.shape[1] != 2 or len(self.vertices) < 3:
            raise ValueError("PolygonObstacle requires an array of shape (N, 2) with N >= 3.")

    def _edges(self) -> list[tuple[np.ndarray, np.ndarray]]:
        return [
            (self.vertices[i], self.vertices[(i + 1) % len(self.vertices)])
            for i in range(len(self.vertices))
        ]

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        best_vector: np.ndarray | None = None
        best_distance = math.inf
        for start, end in self._edges():
            point, _ = _closest_point_on_segment(position, start, end)
            vector = point - position
            distance = _norm(vector)
            if distance < best_distance:
                best_vector = vector
                best_distance = distance
        if best_vector is None:
            return np.array([1e6, 0.0], dtype=float)
        return best_vector

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        samples: list[np.ndarray] = []
        window_fraction = max(0.05, min(0.45, angular_window / math.pi))
        per_edge = max(3, num_samples // len(self.vertices))
        for start, end in self._edges():
            _, t_clamped = _closest_point_on_segment(position, start, end)
            if per_edge <= 1:
                t_values = [t_clamped]
            else:
                t_min = max(0.0, t_clamped - window_fraction)
                t_max = min(1.0, t_clamped + window_fraction)
                t_values = np.linspace(t_min, t_max, per_edge)
            edge = end - start
            for t_value in t_values:
                boundary_point = start + t_value * edge
                samples.append(boundary_point - position)
        return samples

    def _contains_point(self, position: np.ndarray) -> bool:
        x = float(position[0])
        y = float(position[1])
        inside = False
        for start, end in self._edges():
            x1, y1 = float(start[0]), float(start[1])
            x2, y2 = float(end[0]), float(end[1])
            if abs(y2 - y1) < EPS:
                continue
            intersects = ((y1 > y) != (y2 > y))
            if not intersects:
                continue
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x_cross >= x:
                inside = not inside
        return inside

    def clearance(self, position: np.ndarray) -> float:
        min_distance = math.inf
        for start, end in self._edges():
            point, _ = _closest_point_on_segment(position, start, end)
            min_distance = min(min_distance, _norm(point - position))
        if self._contains_point(position):
            return -min_distance
        return min_distance

    def ray_intersection_distance(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        max_range: float,
    ) -> float | None:
        best_t: float | None = None
        for start, end in self._edges():
            edge = end - start
            denom = _cross2(direction, edge)
            if abs(denom) < EPS:
                continue
            rel = start - position
            t = _cross2(rel, edge) / denom
            u = _cross2(rel, direction) / denom
            if EPS <= t <= max_range and 0.0 <= u <= 1.0:
                if best_t is None or t < best_t:
                    best_t = t
        return best_t

    def set_time(self, time_s: float) -> None:
        _ = time_s

    def snapshot(self) -> dict[str, object]:
        return {
            "kind": "polygon",
            "vertices": [[float(vertex[0]), float(vertex[1])] for vertex in self.vertices],
        }
