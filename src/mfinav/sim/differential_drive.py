from __future__ import annotations

from pathlib import Path
import csv

import numpy as np

from ..config.simulation import SimulationConfig
from ..models.differential_drive import DifferentialDriveModel, DifferentialDriveState
from ..navigators.base import Navigator
from ..obstacles.base import Obstacle
from ..utils.math2d import _norm


def _history_row(
    step: int,
    time_s: float,
    state: DifferentialDriveState,
    guidance: np.ndarray,
    goal_distance: float,
    obstacle_distance: float,
    signed_clearance: float,
) -> dict[str, float]:
    velocity = state.velocity
    return {
        "step": float(step),
        "time": float(time_s),
        "x": float(state.position[0]),
        "y": float(state.position[1]),
        "vx": float(velocity[0]),
        "vy": float(velocity[1]),
        "ax": float(guidance[0]),
        "ay": float(guidance[1]),
        "heading": float(state.heading),
        "linear_speed": float(state.linear_speed),
        "angular_speed": float(state.angular_speed),
        "goal_distance": goal_distance,
        "obstacle_distance": obstacle_distance,
        "signed_clearance": signed_clearance,
    }


def simulate_differential_drive(
    state: DifferentialDriveState,
    goal: np.ndarray,
    obstacle: Obstacle,
    config: SimulationConfig | None = None,
    navigator: Navigator | None = None,
    model: DifferentialDriveModel | None = None,
) -> list[dict[str, float]]:
    cfg = config or SimulationConfig()
    if navigator is None:
        raise ValueError("simulate_differential_drive requires an explicit navigator.")

    active_model = model or DifferentialDriveModel(cfg)
    history: list[dict[str, float]] = []

    for step in range(cfg.steps):
        time_s = step * cfg.dt
        if hasattr(obstacle, "set_time"):
            obstacle.set_time(time_s)
        goal_vector = goal - state.position
        observation = navigator.sensing.observe(state, obstacle)
        guidance = np.asarray(navigator.command(state, goal, obstacle), dtype=float)

        history.append(
            _history_row(
                step,
                time_s,
                state,
                guidance,
                _norm(goal_vector),
                observation.distance_to_obstacle,
                observation.signed_clearance,
            )
        )

        command = active_model.guidance_to_command(state, guidance)
        state = active_model.step(state, command, cfg.dt)

        if _norm(goal - state.position) < 0.15 and abs(state.linear_speed) < 0.05:
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
