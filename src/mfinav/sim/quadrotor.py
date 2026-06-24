from __future__ import annotations

from pathlib import Path
import csv

import numpy as np

from ..config.simulation import SimulationConfig
from ..models.quadrotor import QuadrotorModel, QuadrotorState
from ..navigators.base import Navigator
from ..obstacles.base import Obstacle
from ..utils.math2d import _norm


def _history_row(
    step: int,
    time_s: float,
    state: QuadrotorState,
    diagnostics: dict[str, np.ndarray | float],
    goal_distance: float,
    obstacle_distance: float,
    signed_clearance: float,
) -> dict[str, float]:
    guidance = np.asarray(diagnostics["guidance"], dtype=float)
    acceleration = np.asarray(diagnostics["acceleration"], dtype=float)
    torque = np.asarray(diagnostics["torque"], dtype=float)
    rotor_speed = np.asarray(diagnostics["rotor_speed"], dtype=float)
    return {
        "step": float(step),
        "time": float(time_s),
        "x": float(state.position[0]),
        "y": float(state.position[1]),
        "z": float(state.position[2]),
        "vx": float(state.velocity[0]),
        "vy": float(state.velocity[1]),
        "vz": float(state.velocity[2]),
        "ax": float(acceleration[0]),
        "ay": float(acceleration[1]),
        "az": float(acceleration[2]),
        "ux": float(guidance[0]),
        "uy": float(guidance[1]),
        "uz": float(guidance[2]),
        "wx": float(state.angular_velocity[0]),
        "wy": float(state.angular_velocity[1]),
        "wz": float(state.angular_velocity[2]),
        "tx": float(torque[0]),
        "ty": float(torque[1]),
        "tz": float(torque[2]),
        "thrust_total": float(state.thrust_total),
        "rotor0": float(rotor_speed[0]),
        "rotor1": float(rotor_speed[1]),
        "rotor2": float(rotor_speed[2]),
        "rotor3": float(rotor_speed[3]),
        "goal_distance": goal_distance,
        "obstacle_distance": obstacle_distance,
        "signed_clearance": signed_clearance,
    }


def simulate_quadrotor(
    state: QuadrotorState,
    goal: np.ndarray,
    obstacle: Obstacle,
    config: SimulationConfig | None = None,
    navigator: Navigator | None = None,
    model: QuadrotorModel | None = None,
) -> list[dict[str, float]]:
    cfg = config or SimulationConfig()
    if navigator is None:
        raise ValueError("simulate_quadrotor requires an explicit navigator.")

    active_model = model or QuadrotorModel(cfg)
    history: list[dict[str, float]] = []

    for step in range(cfg.steps):
        time_s = step * cfg.dt
        if hasattr(obstacle, "set_time"):
            obstacle.set_time(time_s)
        goal_vector = goal - state.position
        observation = navigator.sensing.observe(state, obstacle)
        guidance = np.asarray(navigator.command(state, goal, obstacle), dtype=float)

        diagnostics_stub = {
            "guidance": guidance,
            "acceleration": np.zeros(3, dtype=float),
            "torque": np.zeros(3, dtype=float),
            "rotor_speed": np.zeros(4, dtype=float),
        }
        history.append(
            _history_row(
                step,
                time_s,
                state,
                diagnostics_stub,
                _norm(goal_vector),
                observation.distance_to_obstacle,
                observation.signed_clearance,
            )
        )

        state, diagnostics = active_model.step(state, guidance, cfg.dt)
        history[-1]["ax"] = float(np.asarray(diagnostics["acceleration"], dtype=float)[0])
        history[-1]["ay"] = float(np.asarray(diagnostics["acceleration"], dtype=float)[1])
        history[-1]["az"] = float(np.asarray(diagnostics["acceleration"], dtype=float)[2])
        history[-1]["tx"] = float(np.asarray(diagnostics["torque"], dtype=float)[0])
        history[-1]["ty"] = float(np.asarray(diagnostics["torque"], dtype=float)[1])
        history[-1]["tz"] = float(np.asarray(diagnostics["torque"], dtype=float)[2])
        history[-1]["rotor0"] = float(np.asarray(diagnostics["rotor_speed"], dtype=float)[0])
        history[-1]["rotor1"] = float(np.asarray(diagnostics["rotor_speed"], dtype=float)[1])
        history[-1]["rotor2"] = float(np.asarray(diagnostics["rotor_speed"], dtype=float)[2])
        history[-1]["rotor3"] = float(np.asarray(diagnostics["rotor_speed"], dtype=float)[3])

        if _norm(goal - state.position) < cfg.quadrotor_goal_tolerance and _norm(state.velocity) < cfg.quadrotor_velocity_tolerance:
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
