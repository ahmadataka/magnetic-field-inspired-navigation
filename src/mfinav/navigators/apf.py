from __future__ import annotations

import numpy as np

from ..config.simulation import SimulationConfig
from ..models.double_integrator import DoubleIntegratorState
from ..obstacles.base import Obstacle
from ..sensing.local import LocalSensingModel
from ..utils.math2d import EPS


class ArtificialPotentialFieldNavigator:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config
        self.sensing = LocalSensingModel(config)

    def command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        obstacle: Obstacle,
    ) -> np.ndarray:
        goal_vector = goal - state.position
        attractive = self.config.kp_goal * goal_vector - self.config.kd_goal * state.velocity
        observation = self.sensing.observe(state, obstacle)
        distance = observation.distance_to_obstacle
        repulsive = np.zeros(2, dtype=float)
        if distance <= self.config.r_la:
            repulsive = -self.config.c_field * (1.0 / max(distance, EPS) - 1.0 / self.config.r_la) * (
                observation.used_obstacle_vector / max(distance**2, EPS)
            )
        return attractive + repulsive
