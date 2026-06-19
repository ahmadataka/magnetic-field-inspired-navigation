from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from ..config.simulation import SimulationConfig
from ..obstacles.base import Obstacle
from ..models.double_integrator import DoubleIntegratorState
from ..utils.math2d import _norm


@dataclass
class LocalSensingObservation:
    closest_obstacle_vector: np.ndarray
    distance_to_obstacle: float
    sensed_vectors: list[np.ndarray]
    averaged_obstacle_vector: np.ndarray
    used_obstacle_vector: np.ndarray
    signed_clearance: float


class LocalSensingModel:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def observe(self, state: DoubleIntegratorState, obstacle: Obstacle) -> LocalSensingObservation:
        if self.config.sensing_mode == "raycast" and hasattr(obstacle, "raycast_vectors"):
            sensed_vectors = obstacle.raycast_vectors(
                state.position,
                self.config.sensing_samples_per_obstacle,
                self.config.sensor_range,
            )
        elif hasattr(obstacle, "sensed_vectors"):
            sensed_vectors = obstacle.sensed_vectors(
                state.position,
                self.config.sensing_samples_per_obstacle,
                self.config.sensing_angular_window,
            )
        else:
            sensed_vectors = [obstacle.closest_vector(state.position)]
        if not sensed_vectors:
            fallback = np.array([self.config.sensor_range * 10.0, 0.0], dtype=float)
            return LocalSensingObservation(
                closest_obstacle_vector=fallback,
                distance_to_obstacle=math.inf,
                sensed_vectors=[],
                averaged_obstacle_vector=fallback,
                used_obstacle_vector=fallback,
                signed_clearance=obstacle.clearance(state.position),
            )
        distances = [_norm(vec) for vec in sensed_vectors]
        min_index = int(np.argmin(distances))
        dist_to_obs = sensed_vectors[min_index]
        min_distance = distances[min_index]
        close_vectors = [
            vec for vec, distance in zip(sensed_vectors, distances) if distance <= min_distance + self.config.delta_r
        ]
        averaged_vector = np.mean(np.array(close_vectors, dtype=float), axis=0)
        if _norm(averaged_vector) < min_distance:
            used_vector = averaged_vector
        else:
            used_vector = dist_to_obs
        return LocalSensingObservation(
            closest_obstacle_vector=dist_to_obs,
            distance_to_obstacle=_norm(used_vector),
            sensed_vectors=sensed_vectors,
            averaged_obstacle_vector=averaged_vector,
            used_obstacle_vector=used_vector,
            signed_clearance=obstacle.clearance(state.position),
        )
