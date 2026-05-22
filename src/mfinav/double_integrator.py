from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math
from typing import Protocol

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

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        delta = position - self.center
        base_angle = math.atan2(delta[1], delta[0])
        samples: list[np.ndarray] = []
        if num_samples <= 1:
            angles = [base_angle]
        else:
            angles = np.linspace(base_angle - angular_window, base_angle + angular_window, num_samples)
        for angle in angles:
            surface_point = self.center + self.radius * np.array([math.cos(angle), math.sin(angle)], dtype=float)
            samples.append(surface_point - position)
        return samples


class Obstacle(Protocol):
    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        ...


@dataclass
class ObstacleCollection:
    obstacles: list[CircleObstacle]

    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        if not self.obstacles:
            return np.array([1e6, 0.0], dtype=float)
        closest = self.obstacles[0].closest_vector(position)
        min_distance = _norm(closest)
        for obstacle in self.obstacles[1:]:
            candidate = obstacle.closest_vector(position)
            distance = _norm(candidate)
            if distance < min_distance:
                closest = candidate
                min_distance = distance
        return closest

    def sensed_vectors(self, position: np.ndarray, num_samples: int, angular_window: float) -> list[np.ndarray]:
        vectors: list[np.ndarray] = []
        for obstacle in self.obstacles:
            vectors.extend(obstacle.sensed_vectors(position, num_samples, angular_window))
        return vectors


@dataclass
class DoubleIntegratorState:
    position: np.ndarray
    velocity: np.ndarray


@dataclass
class LocalSensingObservation:
    closest_obstacle_vector: np.ndarray
    distance_to_obstacle: float
    sensed_vectors: list[np.ndarray]
    averaged_obstacle_vector: np.ndarray
    used_obstacle_vector: np.ndarray


@dataclass
class SimulationConfig:
    dt: float = 0.02
    steps: int = 6000
    kp_goal: float = 0.04
    kd_goal: float = 0.5
    kd_speed: float = 0.5
    kp_goal_relaxed: float = 0.04
    kp_geom: float = 5.0
    speed_limit: float = 1.5
    magni_bound: float = 2.5
    r_l: float = 4.0
    r_la: float = 2.5
    c_field: float = 12.0
    c_perp: float = 35.0
    delta_r: float = 0.5
    epsilon_current: float = 3e-6
    sensing_samples_per_obstacle: int = 15
    sensing_angular_window: float = 1.0
    goal_relaxation: bool = True
    use_legacy_goal_relaxation: bool = False
    max_acceleration: float = 4.0
    max_speed_norm: float = 2.0


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
        if goal_mag < EPS or obs_mag >= self.config.r_l or obs_mag < EPS:
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
        observation: LocalSensingObservation,
    ) -> np.ndarray:
        goal_vector = goal - state.position
        goal_mag = _norm(goal_vector)
        if goal_mag < EPS:
            return np.zeros(2, dtype=float)

        if self.config.use_legacy_goal_relaxation:
            kp_scale = self._goal_weight(goal_vector, observation.closest_obstacle_vector)
        else:
            kp_scale = 1.0

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


class LocalSensingModel:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def observe(self, state: DoubleIntegratorState, obstacle: Obstacle) -> LocalSensingObservation:
        if hasattr(obstacle, "sensed_vectors"):
            sensed_vectors = obstacle.sensed_vectors(
                state.position,
                self.config.sensing_samples_per_obstacle,
                self.config.sensing_angular_window,
            )
        else:
            sensed_vectors = [obstacle.closest_vector(state.position)]
        distances = [_norm(vec) for vec in sensed_vectors]
        min_index = int(np.argmin(distances))
        dist_to_obs = sensed_vectors[min_index]
        min_distance = distances[min_index]

        close_vectors = [
            vec for vec, distance in zip(sensed_vectors, distances) if distance <= min_distance + self.config.delta_r
        ]
        averaged_vector = np.mean(np.array(close_vectors, dtype=float), axis=0)

        # Section 4.4 paper heuristic:
        # for concave/non-unique closest-point cases, the averaged vector is
        # shorter than the single closest measurement and is therefore preferred.
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
        )


class BoundaryFollowingField:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def command(self, state: DoubleIntegratorState, observation: LocalSensingObservation) -> np.ndarray:
        distance = observation.distance_to_obstacle
        if distance > self.config.r_l:
            return np.zeros(2, dtype=float)

        speed = _norm(state.velocity)
        agent_cur = _unit(state.velocity) if speed > EPS else np.zeros(2, dtype=float)
        if _norm(agent_cur) < EPS:
            return np.zeros(2, dtype=float)

        dist_to_obs = observation.used_obstacle_vector
        dist_from_obs = -dist_to_obs
        dist_from_obs_hat = _unit(dist_from_obs)

        # Approximate the boundary-following field by projecting the motion
        # direction onto the local obstacle tangent.
        obs_cur = agent_cur - float(np.dot(agent_cur, dist_from_obs_hat)) * dist_from_obs_hat
        obs_cur_mag = _norm(obs_cur)
        if obs_cur_mag < self.config.epsilon_current:
            obs_cur = _perp_left(dist_from_obs_hat)
        else:
            obs_cur = obs_cur / obs_cur_mag

        sigma_b = max(0.0, min(1.0, (self.config.r_l - distance) / max(self.config.r_l - self.config.r_la, EPS)))
        b_field = _perp_left(obs_cur) * speed / max(distance, EPS)
        agent_perp = _perp_left(agent_cur)
        return sigma_b * self.config.c_field * agent_perp * float(np.dot(agent_perp, b_field))


class CollisionAvoidanceField:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def command(self, state: DoubleIntegratorState, observation: LocalSensingObservation) -> np.ndarray:
        distance = observation.distance_to_obstacle
        if distance > self.config.r_la:
            return np.zeros(2, dtype=float)

        speed = _norm(state.velocity)
        agent_cur = _unit(state.velocity) if speed > EPS else np.zeros(2, dtype=float)
        dist_to_obs = observation.used_obstacle_vector

        sigma_a = max(0.0, min(1.0, (self.config.r_la - distance) / max(self.config.r_la, EPS)))
        repulsion = -sigma_a * self.config.c_perp * (1.0 / max(distance, EPS) - 1.0 / self.config.r_la) * (
            dist_to_obs / max(distance, EPS)
        )
        if _norm(agent_cur) > EPS:
            repulsion = repulsion - float(np.dot(repulsion, agent_cur)) * agent_cur
        return repulsion


class ReferenceNavigator:
    def __init__(self, config: SimulationConfig) -> None:
        self.sensing = LocalSensingModel(config)
        self.goal_controller = GoalRelaxationController(config)
        self.boundary_following = BoundaryFollowingField(config)
        self.collision_avoidance = CollisionAvoidanceField(config)

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
        return goal_cmd + boundary_cmd + collision_cmd


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


@dataclass
class BenchmarkScenario:
    name: str
    start: np.ndarray
    goal: np.ndarray
    obstacles: ObstacleCollection
    description: str


def make_default_scenarios() -> list[BenchmarkScenario]:
    return [
        BenchmarkScenario(
            name="single_convex",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([10.0, 1.5], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[CircleObstacle(center=np.array([5.0, 0.0], dtype=float), radius=1.0)]
            ),
            description="Single convex obstacle between start and goal.",
        ),
        BenchmarkScenario(
            name="convex_cluster",
            start=np.array([0.0, -1.0], dtype=float),
            goal=np.array([11.0, 1.5], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    CircleObstacle(center=np.array([4.0, 0.0], dtype=float), radius=1.0),
                    CircleObstacle(center=np.array([6.0, 1.2], dtype=float), radius=1.1),
                    CircleObstacle(center=np.array([7.5, -0.8], dtype=float), radius=0.9),
                ]
            ),
            description="Several separated convex obstacles requiring repeated boundary following.",
        ),
        BenchmarkScenario(
            name="nonconvex_u_shape",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([10.5, 0.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    CircleObstacle(center=np.array([4.5, -2.0], dtype=float), radius=0.9),
                    CircleObstacle(center=np.array([4.5, -0.7], dtype=float), radius=0.9),
                    CircleObstacle(center=np.array([4.5, 0.7], dtype=float), radius=0.9),
                    CircleObstacle(center=np.array([4.5, 2.0], dtype=float), radius=0.9),
                    CircleObstacle(center=np.array([6.0, -2.0], dtype=float), radius=0.9),
                    CircleObstacle(center=np.array([7.5, -2.0], dtype=float), radius=0.9),
                ]
            ),
            description="Non-convex U-shaped obstacle approximation with a top opening.",
        ),
    ]


def compute_metrics(history: list[dict[str, float]], goal: np.ndarray, success_radius: float = 0.3) -> dict[str, float]:
    path_length = 0.0
    for prev, cur in zip(history, history[1:]):
        path_length += math.hypot(cur["x"] - prev["x"], cur["y"] - prev["y"])

    final = history[-1]
    final_goal_distance = final["goal_distance"]
    min_clearance = min(row["obstacle_distance"] for row in history)
    collision = 1.0 if min_clearance <= 1e-3 else 0.0
    speed_samples = [math.hypot(row["vx"], row["vy"]) for row in history]
    mean_speed = float(sum(speed_samples) / max(len(speed_samples), 1))
    first_success_step = next((row["step"] for row in history if row["goal_distance"] <= success_radius), None)
    time_to_goal = float(first_success_step) if first_success_step is not None else math.inf
    realized_displacement = _norm(
        np.array([history[-1]["x"], history[-1]["y"]], dtype=float) - np.array([history[0]["x"], history[0]["y"]], dtype=float)
    )
    path_efficiency = realized_displacement / max(path_length, EPS)
    return {
        "steps": float(len(history)),
        "path_length": path_length,
        "final_goal_distance": final_goal_distance,
        "min_clearance": min_clearance,
        "mean_speed": mean_speed,
        "success": 1.0 if final_goal_distance <= success_radius and collision == 0.0 else 0.0,
        "collision": collision,
        "time_to_goal_steps": time_to_goal,
        "path_efficiency": path_efficiency,
    }


def _clip_norm(vec: np.ndarray, limit: float) -> np.ndarray:
    magnitude = _norm(vec)
    if magnitude <= limit or magnitude < EPS:
        return vec
    return vec * (limit / magnitude)


def simulate(
    state: DoubleIntegratorState,
    goal: np.ndarray,
    obstacle: Obstacle,
    config: SimulationConfig | None = None,
    navigator: ReferenceNavigator | ArtificialPotentialFieldNavigator | None = None,
) -> list[dict[str, float]]:
    cfg = config or SimulationConfig()
    active_navigator = navigator or ReferenceNavigator(cfg)
    history: list[dict[str, float]] = []

    for step in range(cfg.steps):
        goal_vector = goal - state.position
        observation = active_navigator.sensing.observe(state, obstacle)
        accel = active_navigator.command(state, goal, obstacle)
        accel = _clip_norm(accel, cfg.max_acceleration)

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
                "obstacle_distance": observation.distance_to_obstacle,
            }
        )

        state.velocity = state.velocity + accel * cfg.dt
        state.velocity = _clip_norm(state.velocity, cfg.max_speed_norm)
        state.position = state.position + state.velocity * cfg.dt

        if _norm(goal - state.position) < 0.15 and _norm(state.velocity) < 0.1:
            break
        if observation.distance_to_obstacle <= 1e-3:
            break

    return history


def write_history_csv(history: list[dict[str, float]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(history[0].keys()))
        writer.writeheader()
        writer.writerows(history)
