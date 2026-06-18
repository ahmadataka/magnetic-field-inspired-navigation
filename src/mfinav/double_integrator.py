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


def _signed_angle(from_vec: np.ndarray, to_vec: np.ndarray) -> float:
    from_hat = _unit(from_vec)
    to_hat = _unit(to_vec)
    if _norm(from_hat) < EPS or _norm(to_hat) < EPS:
        return 0.0
    return math.atan2(
        float(from_hat[0] * to_hat[1] - from_hat[1] * to_hat[0]),
        float(np.dot(from_hat, to_hat)),
    )


def _embed_2d(vec: np.ndarray) -> np.ndarray:
    return np.array([float(vec[0]), float(vec[1]), 0.0], dtype=float)


def _project_2d(vec: np.ndarray) -> np.ndarray:
    return np.array([float(vec[0]), float(vec[1])], dtype=float)


def _skew3(vec: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [0.0, -float(vec[2]), float(vec[1])],
            [float(vec[2]), 0.0, -float(vec[0])],
            [-float(vec[1]), float(vec[0]), 0.0],
        ],
        dtype=float,
    )


def _cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _closest_point_on_segment(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> tuple[np.ndarray, float]:
    segment = end - start
    denom = float(np.dot(segment, segment))
    if denom < EPS:
        return start.copy(), 0.0
    t = float(np.dot(point - start, segment) / denom)
    t_clamped = max(0.0, min(1.0, t))
    return start + t_clamped * segment, t_clamped


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

    def clearance(self, position: np.ndarray) -> float:
        return _norm(position - self.center) - self.radius

    def ray_intersection_distance(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        max_range: float,
    ) -> float | None:
        rel = position - self.center
        b = 2.0 * float(np.dot(direction, rel))
        c = float(np.dot(rel, rel) - self.radius**2)
        disc = b * b - 4.0 * c
        if disc < 0.0:
            return None
        sqrt_disc = math.sqrt(disc)
        candidates = [(-b - sqrt_disc) / 2.0, (-b + sqrt_disc) / 2.0]
        valid = [t for t in candidates if EPS <= t <= max_range]
        if not valid:
            return None
        return min(valid)


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


class Obstacle(Protocol):
    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        ...

    def clearance(self, position: np.ndarray) -> float:
        ...

    def ray_intersection_distance(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        max_range: float,
    ) -> float | None:
        ...


@dataclass
class ObstacleCollection:
    obstacles: list[Obstacle]

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

    def raycast_vectors(self, position: np.ndarray, num_samples: int, max_range: float) -> list[np.ndarray]:
        vectors: list[np.ndarray] = []
        angles = np.linspace(-math.pi, math.pi, max(8, num_samples), endpoint=False)
        for angle in angles:
            direction = np.array([math.cos(angle), math.sin(angle)], dtype=float)
            best_t: float | None = None
            for obstacle in self.obstacles:
                candidate = obstacle.ray_intersection_distance(position, direction, max_range)
                if candidate is not None and (best_t is None or candidate < best_t):
                    best_t = candidate
            if best_t is not None:
                vectors.append(direction * best_t)
        return vectors

    def clearance(self, position: np.ndarray) -> float:
        if not self.obstacles:
            return math.inf
        return min(obstacle.clearance(position) for obstacle in self.obstacles)


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
    signed_clearance: float


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
    safety_clearance: float = 0.05
    goal_mode: str = "hybrid"
    field_mode: str = "pragmatic"
    sensing_mode: str = "analytic"
    sensor_range: float = 6.0
    goal_relaxation_mode: str = "legacy"


def make_paper_pd_config() -> SimulationConfig:
    return SimulationConfig(
        kp_goal=0.04,
        kd_goal=0.5,
        kd_speed=0.5,
        kp_goal_relaxed=0.04,
        kp_geom=5.0,
        speed_limit=1.5,
        magni_bound=2.5,
        r_l=3.0,
        r_la=2.0,
        c_field=10.0,
        c_perp=20.0,
        delta_r=0.5,
        epsilon_current=3e-6,
        goal_relaxation=True,
        use_legacy_goal_relaxation=False,
        goal_mode="pd",
        field_mode="paper",
        sensing_mode="raycast",
        sensor_range=6.0,
        goal_relaxation_mode="paper",
        max_acceleration=math.inf,
        max_speed_norm=math.inf,
    )


def make_paper_geometric_config() -> SimulationConfig:
    cfg = make_paper_pd_config()
    cfg.goal_mode = "geometric"
    return cfg


def make_paper_faithful_config() -> SimulationConfig:
    return make_paper_pd_config()


def make_pragmatic_config() -> SimulationConfig:
    return SimulationConfig(
        goal_mode="hybrid",
        field_mode="pragmatic",
    )


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
        if self.config.goal_relaxation_mode == "paper":
            if goal_mag < EPS or obs_mag < EPS:
                return 1.0
            if self.initial_goal_distance is None:
                self.initial_goal_distance = goal_mag
            self.min_distance_to_goal = min(self.min_distance_to_goal, goal_mag)
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

        if self.config.goal_relaxation:
            kp_scale = self._goal_weight(goal_vector, observation.closest_obstacle_vector)
        else:
            kp_scale = 1.0

        if self.config.goal_mode == "pd":
            return kp_scale * self.config.kp_goal * goal_vector - self.config.kd_goal * state.velocity

        if self.config.goal_mode == "geometric":
            speed = _norm(state.velocity)
            if goal_mag > self.config.magni_bound:
                if speed > EPS:
                    direction = _unit(state.velocity)
                else:
                    direction = _unit(goal_vector)

                speed_error = -(speed - self.config.speed_limit)
                vel_attr = (self.config.kp_goal_relaxed * kp_scale) * speed_error * direction

                if speed > EPS:
                    angle_error = _signed_angle(state.velocity, goal_vector)
                    omega = self.config.kp_geom * kp_scale * angle_error
                    force_add = omega * _perp_left(state.velocity)
                else:
                    force_add = np.zeros(2, dtype=float)
                return vel_attr + force_add

            return kp_scale * self.config.kp_goal * goal_vector - self.config.kd_speed * state.velocity

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

        return kp_scale * self.config.kp_goal * goal_vector - self.config.kd_speed * state.velocity


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
            signed_clearance=obstacle.clearance(state.position),
        )


class BoundaryFollowingField:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config
        self.previous_surface_current: np.ndarray | None = None

    def command(self, state: DoubleIntegratorState, observation: LocalSensingObservation) -> np.ndarray:
        distance = observation.distance_to_obstacle
        if distance > self.config.r_l:
            return np.zeros(2, dtype=float)

        speed = _norm(state.velocity)
        agent_cur = _unit(state.velocity) if speed > EPS else np.zeros(2, dtype=float)

        dist_to_obs = observation.used_obstacle_vector
        dist_from_obs = -dist_to_obs
        dist_from_obs_hat = _unit(dist_from_obs)

        # Approximate the boundary-following field by projecting the motion
        # direction onto the local obstacle tangent.
        obs_cur = agent_cur - float(np.dot(agent_cur, dist_from_obs_hat)) * dist_from_obs_hat
        obs_cur_mag = _norm(obs_cur)
        if self.config.field_mode == "paper":
            if obs_cur_mag >= self.config.epsilon_current:
                obs_cur = obs_cur / obs_cur_mag
                self.previous_surface_current = obs_cur.copy()
            elif self.previous_surface_current is not None:
                obs_cur = self.previous_surface_current.copy()
            else:
                return np.zeros(2, dtype=float)

            velocity_3 = _embed_2d(state.velocity)
            agent_current_3 = _embed_2d(agent_cur)
            obs_current_3 = _embed_2d(obs_cur)
            b_field_3 = _skew3(obs_current_3) @ velocity_3 / max(distance, EPS)
            force_3 = self.config.c_field * (_skew3(agent_current_3) @ b_field_3)
            return _project_2d(force_3)

        if obs_cur_mag < self.config.epsilon_current:
            obs_cur = _perp_left(dist_from_obs_hat)
        else:
            obs_cur = obs_cur / obs_cur_mag

        sigma_b = max(0.0, min(1.0, (self.config.r_l - distance) / max(self.config.r_l - self.config.r_la, EPS)))

        guidance_speed = max(speed, 0.5 * self.config.speed_limit)
        return sigma_b * self.config.c_field * guidance_speed * obs_cur / max(distance, EPS)


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

        if self.config.field_mode == "paper":
            repulsion = -self.config.c_perp * (1.0 / max(distance, EPS) - 1.0 / self.config.r_la) * (
                dist_to_obs / max(distance, EPS)
            )
            if _norm(agent_cur) > EPS:
                repulsion = repulsion - float(np.dot(repulsion, agent_cur)) * agent_cur
            return repulsion

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
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [
                                [4.0, -1.0],
                                [6.0, -1.0],
                                [6.0, 1.0],
                                [4.0, 1.0],
                            ],
                            dtype=float,
                        )
                    )
                ]
            ),
            description="Single convex rectangular obstacle between start and goal.",
        ),
        BenchmarkScenario(
            name="convex_cluster",
            start=np.array([0.0, -1.0], dtype=float),
            goal=np.array([11.0, 1.5], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [
                                [3.2, -1.0],
                                [4.8, -1.0],
                                [4.8, 0.8],
                                [3.2, 0.8],
                            ],
                            dtype=float,
                        )
                    ),
                    PolygonObstacle(
                        vertices=np.array(
                            [
                                [5.6, 0.4],
                                [7.0, 0.1],
                                [7.6, 1.5],
                                [6.2, 1.8],
                            ],
                            dtype=float,
                        )
                    ),
                    PolygonObstacle(
                        vertices=np.array(
                            [
                                [7.1, -1.7],
                                [8.3, -1.1],
                                [7.9, 0.0],
                                [6.8, -0.4],
                            ],
                            dtype=float,
                        )
                    ),
                ]
            ),
            description="Several separated convex polygonal obstacles requiring repeated boundary following.",
        ),
        BenchmarkScenario(
            name="nonconvex_u_shape",
            start=np.array([0.0, 0.0], dtype=float),
            goal=np.array([10.5, 0.0], dtype=float),
            obstacles=ObstacleCollection(
                obstacles=[
                    PolygonObstacle(
                        vertices=np.array(
                            [
                                [4.0, -2.5],
                                [8.0, -2.5],
                                [8.0, -1.2],
                                [5.4, -1.2],
                                [5.4, 1.2],
                                [8.0, 1.2],
                                [8.0, 2.5],
                                [4.0, 2.5],
                            ],
                            dtype=float,
                        )
                    )
                ]
            ),
            description="Non-convex U-shaped polygon obstacle with a right-facing cavity.",
        ),
    ]


def compute_metrics(history: list[dict[str, float]], goal: np.ndarray, success_radius: float = 0.3) -> dict[str, float]:
    path_length = 0.0
    for prev, cur in zip(history, history[1:]):
        path_length += math.hypot(cur["x"] - prev["x"], cur["y"] - prev["y"])

    final = history[-1]
    final_goal_distance = final["goal_distance"]
    min_clearance = min(row["signed_clearance"] for row in history)
    collision = 1.0 if min_clearance <= 0.0 else 0.0
    safety_violation = 1.0 if min_clearance <= 0.05 else 0.0
    speed_samples = [math.hypot(row["vx"], row["vy"]) for row in history]
    mean_speed = float(sum(speed_samples) / max(len(speed_samples), 1))
    first_success_step = next((row["step"] for row in history if row["goal_distance"] <= success_radius), None)
    time_to_goal = float(first_success_step) if first_success_step is not None else math.inf
    goal_reached_once = 1.0 if first_success_step is not None and safety_violation == 0.0 else 0.0
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
        "success": 1.0 if final_goal_distance <= success_radius and safety_violation == 0.0 else 0.0,
        "goal_reached_once": goal_reached_once,
        "collision": collision,
        "safety_violation": safety_violation,
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
                "signed_clearance": observation.signed_clearance,
            }
        )

        state.velocity = state.velocity + accel * cfg.dt
        state.velocity = _clip_norm(state.velocity, cfg.max_speed_norm)
        state.position = state.position + state.velocity * cfg.dt

        if _norm(goal - state.position) < 0.15 and _norm(state.velocity) < 0.1:
            break
        if observation.signed_clearance <= 0.0:
            break

    return history


def write_history_csv(history: list[dict[str, float]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(history[0].keys()))
        writer.writeheader()
        writer.writerows(history)
