from __future__ import annotations

import numpy as np

from ..config.simulation import SimulationConfig
from ..models.double_integrator import DoubleIntegratorState
from ..obstacles.base import Obstacle
from ..sensing.local import LocalSensingModel, LocalSensingObservation
from ..utils.math2d import EPS, _norm, _surface_current_from_observation, _unit
from ..utils.math3d import _cross3
from .mfi import GoalRelaxationController


class BoundaryFollowingField3D:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config
        self.previous_surface_current: np.ndarray | None = None

    def command(self, state: DoubleIntegratorState, observation: LocalSensingObservation) -> np.ndarray:
        distance = observation.distance_to_obstacle
        if distance > self.config.r_l:
            return np.zeros(3, dtype=float)

        speed = _norm(state.velocity)
        agent_cur = _unit(state.velocity) if speed > EPS else np.zeros(3, dtype=float)
        dist_to_obs = observation.used_obstacle_vector
        obs_cur, self.previous_surface_current = _surface_current_from_observation(
            agent_cur,
            dist_to_obs,
            self.config.epsilon_current,
            self.previous_surface_current,
        )
        if obs_cur is None or _norm(agent_cur) < EPS:
            return np.zeros(3, dtype=float)

        magnetic_field = _cross3(obs_cur, state.velocity) / max(distance, EPS)
        return self.config.c_field * _cross3(agent_cur, magnetic_field)


class CollisionAvoidanceField3D:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def command(self, state: DoubleIntegratorState, observation: LocalSensingObservation) -> np.ndarray:
        distance = observation.distance_to_obstacle
        if distance > self.config.r_la:
            return np.zeros(3, dtype=float)

        speed = _norm(state.velocity)
        agent_cur = _unit(state.velocity) if speed > EPS else np.zeros(3, dtype=float)
        if _norm(agent_cur) < EPS:
            return np.zeros(3, dtype=float)

        dist_to_obs = observation.used_obstacle_vector
        repulsion = -self.config.c_perp * (
            1.0 / max(distance, EPS) - 1.0 / self.config.r_la
        ) * (dist_to_obs / max(distance, EPS))
        return repulsion - float(np.dot(repulsion, agent_cur)) * agent_cur


class MagneticFieldNavigator3D:
    def __init__(self, config: SimulationConfig) -> None:
        self.sensing = LocalSensingModel(config)
        self.goal_controller = GoalRelaxationController(config)
        self.boundary_following = BoundaryFollowingField3D(config)
        self.collision_avoidance = CollisionAvoidanceField3D(config)
        self.last_observation: LocalSensingObservation | None = None
        self.last_goal_cmd = np.zeros(3, dtype=float)
        self.last_boundary_cmd = np.zeros(3, dtype=float)
        self.last_collision_cmd = np.zeros(3, dtype=float)
        self.last_total_cmd = np.zeros(3, dtype=float)

    def command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        obstacle: Obstacle,
    ) -> np.ndarray:
        observation = self.sensing.observe(state, obstacle)
        goal_cmd = self.goal_controller.command(state, goal, observation)
        boundary_cmd = self.boundary_following.command(state, observation)
        collision_cmd = self.collision_avoidance.command(state, observation)
        total_cmd = goal_cmd + boundary_cmd + collision_cmd
        self.last_observation = observation
        self.last_goal_cmd = goal_cmd
        self.last_boundary_cmd = boundary_cmd
        self.last_collision_cmd = collision_cmd
        self.last_total_cmd = total_cmd
        return total_cmd
