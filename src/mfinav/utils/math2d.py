from __future__ import annotations

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


def _surface_current_from_observation(
    velocity_direction: np.ndarray,
    obstacle_vector: np.ndarray,
    epsilon_current: float,
    previous_surface_current: np.ndarray | None,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    dist_from_obs_hat = _unit(-obstacle_vector)
    obs_cur = velocity_direction - float(np.dot(velocity_direction, dist_from_obs_hat)) * dist_from_obs_hat
    obs_cur_mag = _norm(obs_cur)
    if obs_cur_mag >= epsilon_current:
        obs_cur = obs_cur / obs_cur_mag
        return obs_cur, obs_cur.copy()
    if previous_surface_current is not None:
        return previous_surface_current.copy(), previous_surface_current.copy()
    return None, previous_surface_current
