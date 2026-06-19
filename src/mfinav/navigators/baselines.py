from __future__ import annotations

import math

import numpy as np

from ..config.simulation import SimulationConfig
from ..models.double_integrator import DoubleIntegratorState
from ..obstacles.base import Obstacle
from ..obstacles.circle import CircleObstacle
from ..obstacles.collection import ObstacleCollection
from ..obstacles.polygon import PolygonObstacle
from ..obstacles.prism import PrismObstacle
from ..obstacles.sphere import SphereObstacle
from ..sensing.local import LocalSensingModel, LocalSensingObservation
from ..utils.math2d import EPS, _norm, _unit
from .mfi import GoalRelaxationController
from .mfi3d import GoalRelaxationController3D


def _lift_to_3d(vector: np.ndarray) -> np.ndarray:
    arr = np.asarray(vector, dtype=float)
    if arr.shape == (3,):
        return arr.copy()
    if arr.shape == (2,):
        return np.array([arr[0], arr[1], 0.0], dtype=float)
    raise ValueError(f"Expected a 2D or 3D vector, got shape {arr.shape}.")


def _drop_dimension(vector3: np.ndarray, dim: int) -> np.ndarray:
    return np.asarray(vector3[:dim], dtype=float)


def _nearest_obstacle(position: np.ndarray, obstacle: Obstacle) -> Obstacle:
    if isinstance(obstacle, ObstacleCollection):
        if not obstacle.obstacles:
            return obstacle
        return min(obstacle.obstacles, key=lambda item: _norm(item.closest_vector(position)))
    return obstacle


def _obstacle_center_vector(position: np.ndarray, obstacle: Obstacle, fallback: np.ndarray) -> np.ndarray:
    local_obstacle = _nearest_obstacle(position, obstacle)
    if isinstance(local_obstacle, (CircleObstacle, SphereObstacle)):
        return np.asarray(local_obstacle.center, dtype=float) - position
    if isinstance(local_obstacle, PolygonObstacle):
        return np.mean(local_obstacle.vertices, axis=0) - position
    if isinstance(local_obstacle, PrismObstacle):
        centroid_xy = np.mean(local_obstacle.vertices_xy, axis=0)
        centroid = np.array([centroid_xy[0], centroid_xy[1], 0.5 * (local_obstacle.z_min + local_obstacle.z_max)], dtype=float)
        return centroid - position
    return fallback.copy()


class _GoalTrackingMixin:
    def __init__(self, config: SimulationConfig) -> None:
        self.goal_controller_2d = GoalRelaxationController(config)
        self.goal_controller_3d = GoalRelaxationController3D(config)

    def _goal_command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        observation: LocalSensingObservation,
    ) -> np.ndarray:
        if state.position.shape == (2,):
            return self.goal_controller_2d.command(state, goal, observation)
        return self.goal_controller_3d.command(state, goal, observation)


class HaddadinNavigator(_GoalTrackingMixin):
    def __init__(self, config: SimulationConfig, gain_2d: float = 3.0, gain_3d: float = 10.0) -> None:
        self.config = config
        self.sensing = LocalSensingModel(config)
        self.gain_2d = gain_2d
        self.gain_3d = gain_3d
        self.last_observation: LocalSensingObservation | None = None
        self.last_goal_cmd = np.zeros(3, dtype=float)
        self.last_avoidance_cmd = np.zeros(3, dtype=float)
        self.last_total_cmd = np.zeros(3, dtype=float)
        super().__init__(config)

    def _avoidance_command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        obstacle: Obstacle,
        observation: LocalSensingObservation,
    ) -> np.ndarray:
        dim = len(state.position)
        distance = observation.distance_to_obstacle
        speed = _norm(state.velocity)
        if distance <= EPS or distance > self.config.r_l or speed <= EPS:
            return np.zeros(dim, dtype=float)

        goal_vector = goal - state.position
        goal3 = _lift_to_3d(goal_vector)
        velocity3 = _lift_to_3d(state.velocity)
        obs3 = _lift_to_3d(observation.used_obstacle_vector)
        center3 = _lift_to_3d(_obstacle_center_vector(state.position, obstacle, observation.used_obstacle_vector))

        goal_norm = _norm(goal3)
        center_norm = _norm(center3)
        obs_norm = _norm(obs3)
        if goal_norm <= EPS or center_norm <= EPS or obs_norm <= EPS:
            return np.zeros(dim, dtype=float)

        agent_cur = velocity3 / speed
        goal_hat = goal3 / goal_norm
        dist_from_obs = -obs3
        d_vect = center3 - float(np.dot(center3, goal_hat)) * goal_hat
        transverse_axis = np.cross(d_vect, goal3)
        transverse_norm = _norm(transverse_axis)
        if transverse_norm <= EPS:
            return np.zeros(dim, dtype=float)

        obs_cur = np.cross(dist_from_obs / max(distance, EPS), transverse_axis / transverse_norm)
        magnetic_field = np.cross(obs_cur, velocity3) / max(speed * distance**2, EPS)
        gain = self.gain_2d if dim == 2 else self.gain_3d
        force = gain * np.cross(agent_cur, magnetic_field) * speed
        return _drop_dimension(force, dim)

    def command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        obstacle: Obstacle,
    ) -> np.ndarray:
        observation = self.sensing.observe(state, obstacle)
        goal_cmd = self._goal_command(state, goal, observation)
        avoidance_cmd = self._avoidance_command(state, goal, obstacle, observation)
        total_cmd = goal_cmd + avoidance_cmd
        self.last_observation = observation
        self.last_goal_cmd = _lift_to_3d(goal_cmd)
        self.last_avoidance_cmd = _lift_to_3d(avoidance_cmd)
        self.last_total_cmd = _lift_to_3d(total_cmd)
        return total_cmd


class SabattiniNavigator(_GoalTrackingMixin):
    def __init__(self, config: SimulationConfig, gain_2d: float = 3.0, gain_3d: float = 10.0) -> None:
        self.config = config
        self.sensing = LocalSensingModel(config)
        self.gain_2d = gain_2d
        self.gain_3d = gain_3d
        self.initial_goal_distance: float | None = None
        self.last_observation: LocalSensingObservation | None = None
        self.last_goal_cmd = np.zeros(3, dtype=float)
        self.last_avoidance_cmd = np.zeros(3, dtype=float)
        self.last_total_cmd = np.zeros(3, dtype=float)
        super().__init__(config)

    def _avoidance_command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        observation: LocalSensingObservation,
    ) -> np.ndarray:
        dim = len(state.position)
        distance = observation.distance_to_obstacle
        if distance <= EPS or distance > self.config.r_la:
            return np.zeros(dim, dtype=float)

        goal_vector = goal - state.position
        goal_distance = _norm(goal_vector)
        if self.initial_goal_distance is None:
            self.initial_goal_distance = goal_distance

        agent_vel = state.velocity
        vect_to_obs = observation.used_obstacle_vector
        speed = _norm(agent_vel)
        if speed > EPS:
            agent_cur = agent_vel
        else:
            agent_cur = vect_to_obs

        agent_cur_norm = _norm(agent_cur)
        if agent_cur_norm <= EPS:
            return np.zeros(dim, dtype=float)

        u_t = 0.1 * goal_vector
        w_vect = u_t - float(np.dot(u_t, agent_cur)) * agent_cur / max(agent_cur_norm**2, EPS)
        w_norm = _norm(w_vect)
        if w_norm <= EPS:
            return np.zeros(dim, dtype=float)

        gain = self.gain_2d if dim == 2 else self.gain_3d
        u_g_vect = -gain * w_vect / w_norm

        v_scalar = float(np.dot(agent_vel, vect_to_obs))
        sigma = 1.0 if v_scalar >= 0.0 else 0.0
        sigmoid = 1.0 if v_scalar >= 0.0 else -1.0
        energy = 0.1 * max(self.initial_goal_distance, 0.0) ** 2
        obs_hat = _unit(vect_to_obs)
        damper = -0.05 * sigmoid * energy / max(distance, EPS) * (abs(v_scalar) + math.exp(-abs(v_scalar))) * obs_hat
        return sigma * (u_g_vect + damper)

    def command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        obstacle: Obstacle,
    ) -> np.ndarray:
        observation = self.sensing.observe(state, obstacle)
        goal_cmd = self._goal_command(state, goal, observation)
        avoidance_cmd = self._avoidance_command(state, goal, observation)
        total_cmd = goal_cmd + avoidance_cmd
        self.last_observation = observation
        self.last_goal_cmd = _lift_to_3d(goal_cmd)
        self.last_avoidance_cmd = _lift_to_3d(avoidance_cmd)
        self.last_total_cmd = _lift_to_3d(total_cmd)
        return total_cmd
