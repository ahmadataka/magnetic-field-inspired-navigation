from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config.simulation import SimulationConfig
from ..utils.math2d import EPS, _norm


@dataclass
class DoubleIntegratorState:
    position: np.ndarray
    velocity: np.ndarray


def _clip_norm(vec: np.ndarray, limit: float) -> np.ndarray:
    magnitude = _norm(vec)
    if magnitude <= limit or magnitude < EPS:
        return vec
    return vec * (limit / magnitude)


class DoubleIntegratorModel:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def clip_command(self, command: np.ndarray) -> np.ndarray:
        return _clip_norm(command, self.config.max_acceleration)

    def step(self, state: DoubleIntegratorState, command: np.ndarray, dt: float) -> DoubleIntegratorState:
        accel = self.clip_command(command)
        velocity = state.velocity + accel * dt
        velocity = _clip_norm(velocity, self.config.max_speed_norm)
        position = state.position + velocity * dt
        return DoubleIntegratorState(position=position, velocity=velocity)
