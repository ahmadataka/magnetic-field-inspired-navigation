from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .circle import CircleObstacle
from .polygon import PolygonObstacle
from .collection import ObstacleCollection
from .prism import PrismObstacle
from .sphere import SphereObstacle


def _motion_offset(
    time_s: float,
    velocity: np.ndarray,
    oscillation_amplitude: np.ndarray | None,
    oscillation_frequency: np.ndarray | None,
    oscillation_phase: np.ndarray | None,
) -> np.ndarray:
    offset = time_s * velocity
    if oscillation_amplitude is not None and oscillation_frequency is not None:
        phase = np.zeros_like(oscillation_amplitude) if oscillation_phase is None else oscillation_phase
        offset = offset + oscillation_amplitude * np.sin(oscillation_frequency * time_s + phase)
    return offset


@dataclass
class MovingCircleObstacle:
    initial_center: np.ndarray
    radius: float
    velocity: np.ndarray
    oscillation_amplitude: np.ndarray | None = None
    oscillation_frequency: np.ndarray | None = None
    oscillation_phase: np.ndarray | None = None
    center: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.initial_center = np.asarray(self.initial_center, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)
        if self.oscillation_amplitude is not None:
            self.oscillation_amplitude = np.asarray(self.oscillation_amplitude, dtype=float)
        if self.oscillation_frequency is not None:
            self.oscillation_frequency = np.asarray(self.oscillation_frequency, dtype=float)
        if self.oscillation_phase is not None:
            self.oscillation_phase = np.asarray(self.oscillation_phase, dtype=float)
        self.center = self.initial_center.copy()
        self._shape = CircleObstacle(self.center.copy(), float(self.radius))

    def set_time(self, time_s: float) -> None:
        self.center = self.initial_center + _motion_offset(
            time_s,
            self.velocity,
            self.oscillation_amplitude,
            self.oscillation_frequency,
            self.oscillation_phase,
        )
        self._shape.center = self.center.copy()

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        return self._shape.closest_vector(position)

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        return self._shape.sensed_vectors(position, num_samples, angular_window)

    def clearance(self, position: np.ndarray) -> float:
        return self._shape.clearance(position)

    def ray_intersection_distance(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        max_range: float,
    ) -> float | None:
        return self._shape.ray_intersection_distance(position, direction, max_range)

    def snapshot(self) -> dict[str, object]:
        return {
            "kind": "circle",
            "center": [float(self.center[0]), float(self.center[1])],
            "radius": float(self.radius),
        }


@dataclass
class MovingPolygonObstacle:
    initial_vertices: np.ndarray
    velocity: np.ndarray
    oscillation_amplitude: np.ndarray | None = None
    oscillation_frequency: np.ndarray | None = None
    oscillation_phase: np.ndarray | None = None
    vertices: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.initial_vertices = np.asarray(self.initial_vertices, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)
        if self.oscillation_amplitude is not None:
            self.oscillation_amplitude = np.asarray(self.oscillation_amplitude, dtype=float)
        if self.oscillation_frequency is not None:
            self.oscillation_frequency = np.asarray(self.oscillation_frequency, dtype=float)
        if self.oscillation_phase is not None:
            self.oscillation_phase = np.asarray(self.oscillation_phase, dtype=float)
        self.vertices = self.initial_vertices.copy()
        self._shape = PolygonObstacle(self.vertices.copy())

    def set_time(self, time_s: float) -> None:
        self.vertices = self.initial_vertices + _motion_offset(
            time_s,
            self.velocity,
            self.oscillation_amplitude,
            self.oscillation_frequency,
            self.oscillation_phase,
        )
        self._shape.vertices = self.vertices.copy()

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        return self._shape.closest_vector(position)

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        return self._shape.sensed_vectors(position, num_samples, angular_window)

    def clearance(self, position: np.ndarray) -> float:
        return self._shape.clearance(position)

    def ray_intersection_distance(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        max_range: float,
    ) -> float | None:
        return self._shape.ray_intersection_distance(position, direction, max_range)

    def snapshot(self) -> dict[str, object]:
        return {
            "kind": "polygon",
            "vertices": [[float(vertex[0]), float(vertex[1])] for vertex in self.vertices],
        }


@dataclass
class MovingSphereObstacle:
    initial_center: np.ndarray
    radius: float
    velocity: np.ndarray
    oscillation_amplitude: np.ndarray | None = None
    oscillation_frequency: np.ndarray | None = None
    oscillation_phase: np.ndarray | None = None
    center: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.initial_center = np.asarray(self.initial_center, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)
        if self.oscillation_amplitude is not None:
            self.oscillation_amplitude = np.asarray(self.oscillation_amplitude, dtype=float)
        if self.oscillation_frequency is not None:
            self.oscillation_frequency = np.asarray(self.oscillation_frequency, dtype=float)
        if self.oscillation_phase is not None:
            self.oscillation_phase = np.asarray(self.oscillation_phase, dtype=float)
        self.center = self.initial_center.copy()
        self._shape = SphereObstacle(self.center.copy(), float(self.radius))

    def set_time(self, time_s: float) -> None:
        self.center = self.initial_center + _motion_offset(
            time_s,
            self.velocity,
            self.oscillation_amplitude,
            self.oscillation_frequency,
            self.oscillation_phase,
        )
        self._shape.center = self.center.copy()

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        return self._shape.closest_vector(position)

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        return self._shape.sensed_vectors(position, num_samples, angular_window)

    def clearance(self, position: np.ndarray) -> float:
        return self._shape.clearance(position)

    def ray_intersection_distance(self, position: np.ndarray, direction: np.ndarray, max_range: float) -> float | None:
        return self._shape.ray_intersection_distance(position, direction, max_range)

    def snapshot(self) -> dict[str, object]:
        return {
            "kind": "sphere",
            "center": [float(self.center[0]), float(self.center[1]), float(self.center[2])],
            "radius": float(self.radius),
        }


@dataclass
class MovingPrismObstacle:
    initial_vertices_xy: np.ndarray
    z_min: float
    z_max: float
    velocity: np.ndarray
    oscillation_amplitude: np.ndarray | None = None
    oscillation_frequency: np.ndarray | None = None
    oscillation_phase: np.ndarray | None = None
    vertices_xy: np.ndarray = field(init=False)
    current_z_min: float = field(init=False)
    current_z_max: float = field(init=False)

    def __post_init__(self) -> None:
        self.initial_vertices_xy = np.asarray(self.initial_vertices_xy, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)
        if self.oscillation_amplitude is not None:
            self.oscillation_amplitude = np.asarray(self.oscillation_amplitude, dtype=float)
        if self.oscillation_frequency is not None:
            self.oscillation_frequency = np.asarray(self.oscillation_frequency, dtype=float)
        if self.oscillation_phase is not None:
            self.oscillation_phase = np.asarray(self.oscillation_phase, dtype=float)
        self.vertices_xy = self.initial_vertices_xy.copy()
        self.current_z_min = float(self.z_min)
        self.current_z_max = float(self.z_max)
        self._shape = PrismObstacle(self.vertices_xy.copy(), self.current_z_min, self.current_z_max)

    def set_time(self, time_s: float) -> None:
        offset = _motion_offset(
            time_s,
            self.velocity,
            self.oscillation_amplitude,
            self.oscillation_frequency,
            self.oscillation_phase,
        )
        self.vertices_xy = self.initial_vertices_xy + offset[:2]
        z_shift = float(offset[2]) if len(offset) >= 3 else 0.0
        self.current_z_min = float(self.z_min + z_shift)
        self.current_z_max = float(self.z_max + z_shift)
        self._shape.vertices_xy = self.vertices_xy.copy()
        self._shape.z_min = self.current_z_min
        self._shape.z_max = self.current_z_max

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        return self._shape.closest_vector(position)

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        return self._shape.sensed_vectors(position, num_samples, angular_window)

    def clearance(self, position: np.ndarray) -> float:
        return self._shape.clearance(position)

    def ray_intersection_distance(self, position: np.ndarray, direction: np.ndarray, max_range: float) -> float | None:
        return self._shape.ray_intersection_distance(position, direction, max_range)

    def snapshot(self) -> dict[str, object]:
        return {
            "kind": "prism",
            "vertices_xy": [[float(vertex[0]), float(vertex[1])] for vertex in self.vertices_xy],
            "z_min": float(self.current_z_min),
            "z_max": float(self.current_z_max),
        }


@dataclass
class DynamicObstacleCollection(ObstacleCollection):
    obstacles: list[object]

    def snapshots_over_time(self, time_samples: list[float]) -> list[dict[str, object]]:
        snapshots: list[dict[str, object]] = []
        for time_s in time_samples:
            self.set_time(time_s)
            snapshots.append(self.snapshot())
        return snapshots
