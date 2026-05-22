from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math

import numpy as np


EPS = 1e-9


def _norm(vec: np.ndarray) -> float:
    return float(np.linalg.norm(vec))


def _unit(vec: np.ndarray) -> np.ndarray:
    mag = _norm(vec)
    if mag < EPS:
        return np.zeros_like(vec)
    return vec / mag


def _perp_left(vec: np.ndarray) -> np.ndarray:
    return np.array([-vec[1], vec[0]], dtype=float)


@dataclass
class CircleObstacle:
    center: np.ndarray
    radius: float

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        delta = position - self.center
        distance = _norm(delta)
        if distance < EPS:
            return np.array([self.radius, 0.0], dtype=float)
        return delta * max(distance - self.radius, EPS) / distance


@dataclass
class DoubleIntegratorState:
    position: np.ndarray
    velocity: np.ndarray


@dataclass
class SimulationConfig:
    dt: float = 0.02
    steps: int = 2000
    kp_goal: float = 0.1
    kd_goal: float = 2.0
    kd_speed: float = 1.5
    kp_goal_relaxed: float = 0.1
    kp_geom: float = 5.0
    speed_limit: float = 1.5
    magni_bound: float = 2.5
    bound: float = 1.5
    bound_add: float = 2.5
    constant2: float = 10.0
    constant_add: float = 0.5
    goal_relaxation: bool = True


class GoalRelaxationController:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config
        self.initial_goal_distance: float | None = None
        self.min_distance_to_goal = math.inf

    def _goal_weight(self, goal_vector: np.ndarray, obs_vector: np.ndarray) -> float:
        if not self.config.goal_relaxation:
            return 1.0

        goal_mag = _norm(goal_vector)
        obs_mag = _norm(obs_vector)
        if goal_mag < EPS or obs_mag >= 2.0 * self.config.bound or obs_mag < EPS:
            self.min_distance_to_goal = min(self.min_distance_to_goal, goal_mag)
            return 1.0

        if self.initial_goal_distance is None:
            self.initial_goal_distance = goal_mag
        self.min_distance_to_goal = min(self.min_distance_to_goal, goal_mag)

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
        obstacle_vector: np.ndarray,
    ) -> np.ndarray:
        goal_vector = goal - state.position
        goal_mag = _norm(goal_vector)
        if goal_mag < EPS:
            return np.zeros(2, dtype=float)

        kp_scale = self._goal_weight(goal_vector, obstacle_vector)

        if goal_mag > self.config.magni_bound:
            speed = _norm(state.velocity)
            direction = _unit(state.velocity) if speed > EPS else _unit(goal_vector)
            vel_attr = -(self.config.kp_goal_relaxed * kp_scale) * (speed - self.config.speed_limit) * direction

            target_dir = _unit(goal_vector)
            current_dir = _unit(state.velocity) if speed > EPS else target_dir
            lateral = _perp_left(current_dir)
            signed_heading_error = float(np.dot(lateral, target_dir))
            force_add = self.config.kp_geom * kp_scale * speed * lateral * signed_heading_error
            return vel_attr + force_add

        return self.config.kp_goal * goal_vector - self.config.kd_speed * state.velocity


class MagneticFieldAvoider:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def command(self, state: DoubleIntegratorState, obstacle: CircleObstacle) -> np.ndarray:
        dist_to_obs = obstacle.closest_vector(state.position)
        distance = _norm(dist_to_obs)
        if distance > 2.0 * self.config.bound:
            return np.zeros(2, dtype=float)

        speed = _norm(state.velocity)
        agent_cur = _unit(state.velocity) if speed > EPS else np.zeros(2, dtype=float)
        if _norm(agent_cur) < EPS:
            return np.zeros(2, dtype=float)

        dist_from_obs = -dist_to_obs
        dist_from_obs_hat = _unit(dist_from_obs)

        # Projection of the robot current onto the obstacle tangent plane/line.
        obs_cur = agent_cur - float(np.dot(agent_cur, dist_from_obs_hat)) * dist_from_obs_hat
        obs_cur_mag = _norm(obs_cur)
        if obs_cur_mag < EPS:
            obs_cur = _perp_left(dist_from_obs_hat)
        else:
            obs_cur = obs_cur / obs_cur_mag

        # 2D analogue of the nested cross-product used in the ROS implementation.
        b_field = _perp_left(obs_cur) * speed / max(distance, EPS)
        agent_perp = _perp_left(agent_cur)
        force = self.config.constant2 * agent_perp * float(np.dot(agent_perp, b_field))

        repulsion = np.zeros(2, dtype=float)
        if distance <= self.config.bound_add:
            repulsion = -self.config.constant_add * (1.0 / max(distance, EPS) - 1.0 / self.config.bound_add) * (
                dist_to_obs / max(distance, EPS)
            )
            repulsion = repulsion - float(np.dot(repulsion, agent_cur)) * agent_cur

        return force + repulsion


class ReferenceNavigator:
    def __init__(self, config: SimulationConfig) -> None:
        self.goal_controller = GoalRelaxationController(config)
        self.avoider = MagneticFieldAvoider(config)

    def command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        obstacle: CircleObstacle,
    ) -> np.ndarray:
        obstacle_vector = obstacle.closest_vector(state.position)
        goal_cmd = self.goal_controller.command(state, goal, obstacle_vector)
        avoid_cmd = self.avoider.command(state, obstacle)
        return goal_cmd + avoid_cmd


def simulate(
    state: DoubleIntegratorState,
    goal: np.ndarray,
    obstacle: CircleObstacle,
    config: SimulationConfig | None = None,
) -> list[dict[str, float]]:
    cfg = config or SimulationConfig()
    navigator = ReferenceNavigator(cfg)
    history: list[dict[str, float]] = []

    for step in range(cfg.steps):
        goal_vector = goal - state.position
        obstacle_vector = obstacle.closest_vector(state.position)
        accel = navigator.command(state, goal, obstacle)

        history.append(
            {
                "step": float(step),
                "x": float(state.position[0]),
                "y": float(state.position[1]),
                "vx": float(state.velocity[0]),
                "vy": float(state.velocity[1]),
                "ax": float(accel[0]),
                "ay": float(accel[1]),
                "goal_distance": _norm(goal_vector),
                "obstacle_distance": _norm(obstacle_vector),
            }
        )

        state.velocity = state.velocity + accel * cfg.dt
        state.position = state.position + state.velocity * cfg.dt

        if _norm(goal - state.position) < 0.15 and _norm(state.velocity) < 0.1:
            break

    return history


def write_history_csv(history: list[dict[str, float]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(history[0].keys()))
        writer.writeheader()
        writer.writerows(history)
