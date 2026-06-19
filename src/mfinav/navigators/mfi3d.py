from __future__ import annotations

import math

import numpy as np

from ..config.simulation import SimulationConfig
from ..models.double_integrator import DoubleIntegratorState
from ..obstacles.base import Obstacle
from ..sensing.local import LocalSensingModel, LocalSensingObservation
from ..utils.math2d import EPS, _norm, _skew3, _surface_current_from_observation, _unit
from ..utils.math3d import _cross3


class GoalRelaxationController3D:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config
        self.initial_goal_distance: float | None = None
        self.min_distance_to_goal = math.inf
        self.last_goal_weight = 1.0

    def _goal_weight(self, goal_vector: np.ndarray, obs_vector: np.ndarray) -> float:
        if not self.config.goal_relaxation:
            return 1.0

        goal_mag = _norm(goal_vector)
        obs_mag = _norm(obs_vector)
        if goal_mag < EPS or obs_mag < EPS:
            return 1.0

        if self.initial_goal_distance is None:
            self.initial_goal_distance = goal_mag
        self.min_distance_to_goal = min(self.min_distance_to_goal, goal_mag)

        if self.config.goal_relaxation_mode == "paper":
            if obs_mag >= 2.0 * self.config.r_l:
                return 1.0

            goal_hat = _unit(goal_vector)
            obs_hat = _unit(obs_vector)
            blockage = max(0.0, float(np.dot(goal_hat, obs_hat)))
            proximity = max(0.0, 1.0 - obs_mag / max(2.0 * self.config.r_l, EPS))
            if goal_mag > self.initial_goal_distance:
                regression = 1.0
            else:
                regression = max(0.0, min(1.0, (goal_mag - self.min_distance_to_goal) / max(self.config.r_l, EPS)))
            relax_strength = proximity * blockage * max(regression, 0.35)
            return max(0.2, 1.0 - 0.8 * relax_strength)

        if obs_mag >= self.config.r_l:
            return 1.0

        cos_angle = float(np.dot(goal_vector, obs_vector) / max(goal_mag * obs_mag, EPS))
        cos_angle = max(-1.0, min(1.0, cos_angle))
        weight = 1.0 - cos_angle

        if self.initial_goal_distance is None or goal_mag <= self.initial_goal_distance:
            new_weight = 1.0
        else:
            new_weight = math.exp(-(goal_mag - self.initial_goal_distance) / 0.1)

        weight_exit = math.exp(-(goal_mag - self.min_distance_to_goal) / 0.1)
        distance_scale = 1.0 - math.exp(-obs_mag / 1.5)
        return distance_scale * weight * new_weight * weight_exit

    def command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        observation: LocalSensingObservation,
    ) -> np.ndarray:
        goal_vector = goal - state.position
        goal_mag = _norm(goal_vector)
        if goal_mag < EPS:
            return np.zeros(3, dtype=float)

        kp_scale = self._goal_weight(goal_vector, observation.used_obstacle_vector) if self.config.goal_relaxation else 1.0
        self.last_goal_weight = kp_scale

        if self.config.goal_mode == "pd":
            return kp_scale * self.config.kp_goal * goal_vector - self.config.kd_goal * state.velocity

        if self.config.goal_mode == "geometric":
            speed = _norm(state.velocity)
            goal_hat = _unit(goal_vector)
            if goal_mag > self.config.magni_bound:
                direction = _unit(state.velocity) if speed > EPS else goal_hat
                speed_error = -(speed - self.config.speed_limit)
                vel_attr = (self.config.kp_goal_relaxed * kp_scale) * speed_error * direction

                skew_direction = _skew3(direction)
                force_add = -self.config.kp_geom * kp_scale * max(self.config.speed_limit, speed, 0.25) * (
                    skew_direction @ (skew_direction @ goal_hat)
                )
                return vel_attr + force_add

            desired_speed = kp_scale * min(self.config.speed_limit, goal_mag)
            desired_velocity = desired_speed * goal_hat
            return self.config.kd_speed * (desired_velocity - state.velocity)

        return kp_scale * self.config.kp_goal * goal_vector - self.config.kd_speed * state.velocity


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
        self.goal_controller = GoalRelaxationController3D(config)
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
