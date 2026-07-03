from __future__ import annotations

import csv
import json
import math
import os
import sys
from pathlib import Path

import numpy as np


WEBOTS_HOME = Path(os.environ.get("WEBOTS_HOME", "/Applications/Webots.app/Contents"))
WEBOTS_PYTHON = WEBOTS_HOME / "lib" / "controller" / "python"
if str(WEBOTS_PYTHON) not in sys.path:
    sys.path.insert(0, str(WEBOTS_PYTHON))

PROJECT_ROOT = Path(__file__).resolve().parents[3]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from controller import Supervisor  # type: ignore

from mfinav import (
    DoubleIntegratorState,
    MagneticFieldNavigator3D,
    ObstacleCollection,
    QuadrotorModel,
    QuadrotorState,
    SphereObstacle,
    compute_metrics,
    make_paper_geometric_3d_config,
    make_paper_pd_3d_config,
)


GOAL_TOLERANCE = 0.35
VELOCITY_TOLERANCE = 0.18
DEFAULT_MAX_STEPS = 1800
DEFAULT_DT = 0.032
DEFAULT_INTERNAL_DT = 0.02
HISTORY_ENV = "MFINAV_WEBOTS_HISTORY"
SUMMARY_ENV = "MFINAV_WEBOTS_SUMMARY"
METHOD_ENV = "MFINAV_WEBOTS_METHOD"
AUTO_QUIT_ENV = "MFINAV_WEBOTS_QUIT"
MAX_STEPS_ENV = "MFINAV_WEBOTS_MAX_STEPS"
GOAL_TOL_ENV = "MFINAV_WEBOTS_GOAL_TOLERANCE"
CONFIG_JSON_ENV = "MFINAV_WEBOTS_CONFIG_JSON"

STATIC_OBSTACLES = ObstacleCollection(
    obstacles=[
        SphereObstacle(center=np.array([10.0, 10.0, 10.0], dtype=float), radius=0.5),
    ]
)


def _write_history_csv(history: list[dict[str, float]], output_path: Path) -> None:
    if not history:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(history[0].keys()))
        writer.writeheader()
        writer.writerows(history)


def _write_summary_json(output_path: Path, summary: dict[str, object]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(summary, indent=2))


def _history_row(
    step: int,
    time_s: float,
    state: QuadrotorState,
    guidance: np.ndarray,
    goal_distance: float,
    obstacle_distance: float,
    signed_clearance: float,
) -> dict[str, float]:
    return {
        "step": float(step),
        "time": float(time_s),
        "x": float(state.position[0]),
        "y": float(state.position[1]),
        "z": float(state.position[2]),
        "vx": float(state.velocity[0]),
        "vy": float(state.velocity[1]),
        "vz": float(state.velocity[2]),
        "gx": float(guidance[0]),
        "gy": float(guidance[1]),
        "gz": float(guidance[2]),
        "goal_distance": float(goal_distance),
        "obstacle_distance": float(obstacle_distance),
        "signed_clearance": float(signed_clearance),
    }


def _axis_angle_from_rotation(rotation: np.ndarray) -> list[float]:
    trace = float(np.trace(rotation))
    cos_angle = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    angle = math.acos(cos_angle)
    if abs(angle) < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]

    axis = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ],
        dtype=float,
    )
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-9:
        eigenvalues, eigenvectors = np.linalg.eig(rotation)
        idx = int(np.argmin(np.abs(eigenvalues - 1.0)))
        axis = np.real(eigenvectors[:, idx])
        axis_norm = float(np.linalg.norm(axis))
    axis = axis / max(axis_norm, 1e-9)
    return [float(axis[0]), float(axis[1]), float(axis[2]), float(angle)]


def _apply_env_config_overrides(config) -> None:
    raw = os.environ.get(CONFIG_JSON_ENV)
    if not raw:
        return
    overrides = json.loads(raw)
    for key, value in overrides.items():
        if hasattr(config, key):
            setattr(config, key, value)


def main() -> None:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0 if timestep > 0 else DEFAULT_DT
    max_steps = int(os.environ.get(MAX_STEPS_ENV, str(DEFAULT_MAX_STEPS)))
    history_output = os.environ.get(HISTORY_ENV)
    summary_output = os.environ.get(SUMMARY_ENV)
    auto_quit = os.environ.get(AUTO_QUIT_ENV, "0") == "1"
    goal_tolerance = float(os.environ.get(GOAL_TOL_ENV, str(GOAL_TOLERANCE)))
    internal_dt = float(os.environ.get("MFINAV_WEBOTS_INTERNAL_DT", str(DEFAULT_INTERNAL_DT)))
    method_name = os.environ.get(METHOD_ENV, "paper_geometric_3d")

    if method_name == "waypoint_pd":
        config = make_paper_pd_3d_config()
    elif method_name == "paper_pd_3d":
        config = make_paper_pd_3d_config()
    elif method_name == "paper_geometric_3d":
        config = make_paper_geometric_3d_config()
    else:
        raise ValueError(f"Unsupported Webots quadrotor method: {method_name}")

    config.quadrotor_goal_tolerance = goal_tolerance
    config.quadrotor_velocity_tolerance = VELOCITY_TOLERANCE
    config.max_speed_norm = 1.4
    config.quadrotor_guidance_limit = 8.0
    _apply_env_config_overrides(config)

    navigator = MagneticFieldNavigator3D(config)
    model = QuadrotorModel(config)

    self_node = robot.getSelf()
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")
    goal_node = robot.getFromDef("GOAL")
    if goal_node is None:
        raise RuntimeError("Missing GOAL node in quadrotor Webots arena.")

    start_position = np.asarray(self_node.getPosition(), dtype=float)
    goal_position = np.asarray(goal_node.getPosition(), dtype=float)
    state = QuadrotorState(
        position=start_position.copy(),
        velocity=np.zeros(3, dtype=float),
        rotation=np.eye(3, dtype=float),
        angular_velocity=np.zeros(3, dtype=float),
        thrust_total=config.quadrotor_mass * config.quadrotor_gravity,
        thrust_rate=0.0,
    )

    history: list[dict[str, float]] = []
    summary: dict[str, object] = {
        "status": "unknown",
        "steps": 0,
        "scenario": "webots_quadrotor_static_smoke",
        "method": method_name,
        "goal_tolerance": goal_tolerance,
        "velocity_tolerance": VELOCITY_TOLERANCE,
    }
    exit_code = 0

    for step in range(max_steps):
        if robot.step(timestep) == -1:
            summary["status"] = "webots_stopped"
            break

        time_s = step * dt
        if method_name == "waypoint_pd":
            position_error = goal_position - state.position
            guidance = 1.4 * position_error - 1.8 * state.velocity
            closest_distance = float(np.linalg.norm(STATIC_OBSTACLES.closest_vector(state.position)))
            closest_clearance = float(STATIC_OBSTACLES.clearance(state.position))
        else:
            di_state = DoubleIntegratorState(position=state.position.copy(), velocity=state.velocity.copy())
            guidance = np.asarray(navigator.command(di_state, goal_position, STATIC_OBSTACLES), dtype=float)
            observation = navigator.last_observation
            if observation is None:
                continue
            closest_distance = float(observation.distance_to_obstacle)
            closest_clearance = float(observation.signed_clearance)
        goal_distance = float(np.linalg.norm(goal_position - state.position))
        history.append(
            _history_row(
                step,
                time_s,
                state,
                guidance,
                goal_distance,
                closest_distance,
                closest_clearance,
            )
        )

        remaining_dt = dt
        while remaining_dt > 1e-9:
            step_dt = min(internal_dt, remaining_dt)
            if method_name == "waypoint_pd":
                step_guidance = 1.4 * (goal_position - state.position) - 1.8 * state.velocity
            else:
                step_state = DoubleIntegratorState(position=state.position.copy(), velocity=state.velocity.copy())
                step_guidance = np.asarray(navigator.command(step_state, goal_position, STATIC_OBSTACLES), dtype=float)
            state, _ = model.step(state, step_guidance, step_dt)
            remaining_dt -= step_dt
        translation_field.setSFVec3f([float(state.position[0]), float(state.position[1]), float(state.position[2])])
        rotation_field.setSFRotation(_axis_angle_from_rotation(state.rotation))
        summary["steps"] = step + 1

        if closest_clearance <= 0.0:
            summary["status"] = "collision"
            exit_code = 2
            break

        if goal_distance <= goal_tolerance:
            summary["status"] = "goal_reached"
            break
    else:
        summary["status"] = "timeout"
        exit_code = 1

    if history:
        metrics = compute_metrics(history, goal_position, success_radius=goal_tolerance)
        summary["metrics"] = {key: float(value) for key, value in metrics.items()}
        summary["final_state"] = {
            "x": float(state.position[0]),
            "y": float(state.position[1]),
            "z": float(state.position[2]),
            "speed_norm": float(np.linalg.norm(state.velocity)),
        }
        summary["start"] = [float(start_position[0]), float(start_position[1]), float(start_position[2])]
        summary["goal"] = [float(goal_position[0]), float(goal_position[1]), float(goal_position[2])]
        summary["obstacles"] = STATIC_OBSTACLES.snapshot()["obstacles"]

    if history_output:
        _write_history_csv(history, Path(history_output))
    if summary_output:
        _write_summary_json(Path(summary_output), summary)

    if auto_quit:
        robot.simulationQuit(exit_code)


if __name__ == "__main__":
    main()
