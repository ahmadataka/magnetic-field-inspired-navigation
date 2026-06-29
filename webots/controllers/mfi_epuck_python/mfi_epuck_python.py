from __future__ import annotations

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

from mfinav import DifferentialDriveModel, DifferentialDriveState, PolygonObstacle, ReferenceNavigator, make_paper_geometric_config


GOAL_TOLERANCE = 0.10

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


def main() -> None:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

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

    while robot.step(timestep) != -1:
        goal_position = np.asarray(goal_node.getPosition(), dtype=float)[:2]
        closest_obstacle = None
        closest_distance = math.inf
        for node, size_xy in obstacle_nodes:
            obstacle = _obstacle_from_node(node, size_xy)
            distance = float(np.linalg.norm(obstacle.closest_vector(state.position)))
            if distance < closest_distance:
                closest_distance = distance
                closest_obstacle = obstacle

        if closest_obstacle is None:
            continue

        guidance = np.asarray(navigator.command(state, goal_position, closest_obstacle), dtype=float)
        command = model.guidance_to_command(state, guidance)
        state = model.step(state, command, timestep / 1000.0)
        translation_field.setSFVec3f([float(state.position[0]), float(state.position[1]), 0.035])
        rotation_field.setSFRotation([0.0, 0.0, 1.0, float(state.heading)])

        if float(np.linalg.norm(goal_position - state.position)) <= GOAL_TOLERANCE:
            print("Goal reached in Webots arena.")
            break


if __name__ == "__main__":
    main()
