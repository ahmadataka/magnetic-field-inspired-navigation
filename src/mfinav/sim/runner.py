from __future__ import annotations

from pathlib import Path
import csv

import numpy as np

from ..config.simulation import SimulationConfig
from ..models.double_integrator import DoubleIntegratorModel, DoubleIntegratorState
from ..navigators.apf import ArtificialPotentialFieldNavigator
from ..navigators.mfi import MagneticFieldNavigator, ReferenceNavigator
from ..obstacles.base import Obstacle
from ..utils.math2d import _norm


def simulate(
    state: DoubleIntegratorState,
    goal: np.ndarray,
    obstacle: Obstacle,
    config: SimulationConfig | None = None,
    navigator: ReferenceNavigator | ArtificialPotentialFieldNavigator | None = None,
    model: DoubleIntegratorModel | None = None,
) -> list[dict[str, float]]:
    cfg = config or SimulationConfig()
    active_navigator = navigator or MagneticFieldNavigator(cfg)
    active_model = model or DoubleIntegratorModel(cfg)
    history: list[dict[str, float]] = []

    for step in range(cfg.steps):
        goal_vector = goal - state.position
        observation = active_navigator.sensing.observe(state, obstacle)
        accel = active_navigator.command(state, goal, obstacle)
        accel = active_model.clip_command(accel)

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
