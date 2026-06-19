from __future__ import annotations

import math

import numpy as np

from ..utils.math2d import EPS, _norm


def _point_from_row(row: dict[str, float]) -> np.ndarray:
    coords = [row["x"], row["y"]]
    if "z" in row:
        coords.append(row["z"])
    return np.array(coords, dtype=float)


def _velocity_from_row(row: dict[str, float]) -> np.ndarray:
    coords = [row["vx"], row["vy"]]
    if "vz" in row:
        coords.append(row["vz"])
    return np.array(coords, dtype=float)


def compute_metrics(history: list[dict[str, float]], goal: np.ndarray, success_radius: float = 0.3) -> dict[str, float]:
    path_length = 0.0
    for prev, cur in zip(history, history[1:]):
        path_length += _norm(_point_from_row(cur) - _point_from_row(prev))

    final = history[-1]
    final_goal_distance = final["goal_distance"]
    min_clearance = min(row["signed_clearance"] for row in history)
    collision = 1.0 if min_clearance <= 0.0 else 0.0
    safety_violation = 1.0 if min_clearance <= 0.05 else 0.0
    speed_samples = [_norm(_velocity_from_row(row)) for row in history]
    mean_speed = float(sum(speed_samples) / max(len(speed_samples), 1))
    first_success_step = next((row["step"] for row in history if row["goal_distance"] <= success_radius), None)
    time_to_goal = float(first_success_step) if first_success_step is not None else math.inf
    goal_reached_once = 1.0 if first_success_step is not None and safety_violation == 0.0 else 0.0
    realized_displacement = _norm(_point_from_row(history[-1]) - _point_from_row(history[0]))
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
