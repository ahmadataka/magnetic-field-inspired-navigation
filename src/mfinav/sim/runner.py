from __future__ import annotations

from pathlib import Path
import csv

import numpy as np

from ..config.simulation import SimulationConfig
from ..models.double_integrator import DoubleIntegratorModel, DoubleIntegratorState
from ..navigators.base import Navigator
from ..navigators.mfi import MagneticFieldNavigator
from ..obstacles.base import Obstacle
from ..utils.math2d import _norm


def _history_row(
    step: int,
    time_s: float,
    state: DoubleIntegratorState,
    accel: np.ndarray,
    goal_distance: float,
    obstacle_distance: float,
    signed_clearance: float,
) -> dict[str, float]:
    row = {
        "step": float(step),
        "time": float(time_s),
        "goal_distance": goal_distance,
        "obstacle_distance": obstacle_distance,
        "signed_clearance": signed_clearance,
    }
    axis_names = ("x", "y", "z")
    for idx, axis in enumerate(axis_names[: len(state.position)]):
        row[axis] = float(state.position[idx])
        row[f"v{axis}"] = float(state.velocity[idx])
        row[f"a{axis}"] = float(accel[idx])
    return row


def simulate(
    state: DoubleIntegratorState,
    goal: np.ndarray,
    obstacle: Obstacle,
    config: SimulationConfig | None = None,
    navigator: Navigator | None = None,
    model: DoubleIntegratorModel | None = None,
) -> list[dict[str, float]]:
    cfg = config or SimulationConfig()
    active_navigator = navigator or MagneticFieldNavigator(cfg)
    active_model = model or DoubleIntegratorModel(cfg)
    history: list[dict[str, float]] = []

    for step in range(cfg.steps):
        time_s = step * cfg.dt
        if hasattr(obstacle, "set_time"):
            obstacle.set_time(time_s)
        goal_vector = goal - state.position
        observation = active_navigator.sensing.observe(state, obstacle)
        accel = active_navigator.command(state, goal, obstacle)
        accel = active_model.clip_command(accel)

        history.append(
            _history_row(
                step,
                time_s,
                state,
                accel,
                _norm(goal_vector),
                observation.distance_to_obstacle,
                observation.signed_clearance,
            )
        )

        state = active_model.step(state, accel, cfg.dt)

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
