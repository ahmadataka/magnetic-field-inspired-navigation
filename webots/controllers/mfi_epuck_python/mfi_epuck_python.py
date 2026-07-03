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

from mfinav import DifferentialDriveModel, DifferentialDriveState, PolygonObstacle, ReferenceNavigator, compute_metrics, make_paper_geometric_config


GOAL_TOLERANCE = 0.10
DEFAULT_MAX_STEPS = 2500
DEFAULT_DT = 0.064

OBSTACLE_SPECS = [
    {"def": "OBSTACLE_A", "size": np.array([0.22, 0.22], dtype=float)},
    {"def": "OBSTACLE_B", "size": np.array([0.26, 0.18], dtype=float)},
    {"def": "OBSTACLE_C", "size": np.array([0.18, 0.28], dtype=float)},
]


def _rotation_matrix_2d(theta: float) -> np.ndarray:
    return np.array(
        [
            [math.cos(theta), -math.sin(theta)],
            [math.sin(theta), math.cos(theta)],
        ],
        dtype=float,
    )


def _heading_from_orientation_matrix(orientation: np.ndarray) -> float:
    return math.atan2(float(orientation[3]), float(orientation[0]))


def _box_polygon(center: np.ndarray, yaw: float, size_xy: np.ndarray) -> PolygonObstacle:
    half = 0.5 * size_xy
    corners = np.array(
        [
            [-half[0], -half[1]],
            [half[0], -half[1]],
            [half[0], half[1]],
            [-half[0], half[1]],
        ],
        dtype=float,
    )
    vertices = (corners @ _rotation_matrix_2d(yaw).T) + center
    return PolygonObstacle(vertices=vertices)


def _obstacle_from_node(node, size_xy: np.ndarray) -> PolygonObstacle:
    position = np.asarray(node.getPosition(), dtype=float)
    orientation = np.asarray(node.getOrientation(), dtype=float).reshape(3, 3)
    yaw = _heading_from_orientation_matrix(orientation.ravel())
    return _box_polygon(position[:2], yaw, size_xy)


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
        "goal_distance": float(goal_distance),
        "obstacle_distance": float(obstacle_distance),
        "signed_clearance": float(signed_clearance),
    }


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


def main() -> None:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0 if timestep > 0 else DEFAULT_DT
    max_steps = int(os.environ.get("MFINAV_WEBOTS_MAX_STEPS", str(DEFAULT_MAX_STEPS)))
    history_output = os.environ.get("MFINAV_WEBOTS_HISTORY")
    summary_output = os.environ.get("MFINAV_WEBOTS_SUMMARY")
    auto_quit = os.environ.get("MFINAV_WEBOTS_QUIT", "0") == "1"

    config = make_paper_geometric_config()
    config.max_linear_speed = 0.18
    config.max_angular_speed = 4.0
    config.speed_gain = 1.2
    config.heading_gain = 3.5
    config.min_forward_factor = 0.2

    navigator = ReferenceNavigator(config)
    model = DifferentialDriveModel(config)

    self_node = robot.getSelf()
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")
    goal_node = robot.getFromDef("GOAL")
    obstacle_nodes = []
    for spec in OBSTACLE_SPECS:
        node = robot.getFromDef(spec["def"])
        if node is not None:
            obstacle_nodes.append((node, spec["size"]))

    if goal_node is None or not obstacle_nodes:
        print("Missing GOAL or obstacle DEF nodes in Webots world.")
        return

    initial_position = np.asarray(self_node.getPosition(), dtype=float)
    initial_orientation = np.asarray(self_node.getOrientation(), dtype=float).reshape(3, 3)
    state = DifferentialDriveState(
        position=initial_position[:2].copy(),
        heading=_heading_from_orientation_matrix(initial_orientation.ravel()),
        linear_speed=0.0,
        angular_speed=0.0,
    )
    history: list[dict[str, float]] = []
    summary: dict[str, object] = {
        "status": "unknown",
        "steps": 0,
        "goal_tolerance": GOAL_TOLERANCE,
    }
    exit_code = 0

    for step in range(max_steps):
        if robot.step(timestep) == -1:
            summary["status"] = "webots_stopped"
            break
        time_s = step * dt
        goal_position = np.asarray(goal_node.getPosition(), dtype=float)[:2]
        closest_obstacle = None
        closest_distance = math.inf
        closest_clearance = math.inf
        for node, size_xy in obstacle_nodes:
            obstacle = _obstacle_from_node(node, size_xy)
            distance = float(np.linalg.norm(obstacle.closest_vector(state.position)))
            clearance = float(obstacle.clearance(state.position))
            if distance < closest_distance:
                closest_distance = distance
                closest_obstacle = obstacle
                closest_clearance = clearance

        if closest_obstacle is None:
            continue

        guidance = np.asarray(navigator.command(state, goal_position, closest_obstacle), dtype=float)
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
        command = model.guidance_to_command(state, guidance)
        state = model.step(state, command, dt)
        translation_field.setSFVec3f([float(state.position[0]), float(state.position[1]), 0.035])
        rotation_field.setSFRotation([0.0, 0.0, 1.0, float(state.heading)])

        summary["steps"] = step + 1

        if closest_clearance <= 0.0:
            summary["status"] = "collision"
            exit_code = 2
            print("Collision detected in Webots arena.")
            break

        if goal_distance <= GOAL_TOLERANCE:
            summary["status"] = "goal_reached"
            print("Goal reached in Webots arena.")
            break
    else:
        summary["status"] = "timeout"
        exit_code = 1
        print("Webots arena run reached the step limit without reaching the goal.")

    if history:
        metrics = compute_metrics(history, goal_position, success_radius=GOAL_TOLERANCE)
        summary["metrics"] = {key: float(value) for key, value in metrics.items()}
        summary["final_state"] = {
            "x": float(history[-1]["x"]),
            "y": float(history[-1]["y"]),
            "heading": float(history[-1]["heading"]),
        }
        summary["start"] = [float(initial_position[0]), float(initial_position[1])]
        summary["goal"] = [float(goal_position[0]), float(goal_position[1])]

    if history_output:
        _write_history_csv(history, Path(history_output))
    if summary_output:
        _write_summary_json(Path(summary_output), summary)

    if auto_quit:
        robot.simulationQuit(exit_code)


if __name__ == "__main__":
    main()
