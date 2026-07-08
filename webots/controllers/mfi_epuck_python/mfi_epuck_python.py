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
    ArtificialPotentialFieldNavigator,
    CircleObstacle,
    DifferentialDriveModel,
    DifferentialDriveState,
    HaddadinNavigator,
    LocalSensingModel,
    MovingCircleObstacle,
    MovingPolygonObstacle,
    ObstacleCollection,
    PolygonObstacle,
    ReferenceNavigator,
    SabattiniNavigator,
    compute_metrics,
    make_paper_geometric_config,
    make_paper_pd_config,
)


GOAL_TOLERANCE = 0.10
DEFAULT_MAX_STEPS = 2500
DEFAULT_DT = 0.064
DEFAULT_INTERNAL_DT = 0.02
SCENARIO_JSON_ENV = "MFINAV_WEBOTS_SCENARIO_JSON"
METHOD_ENV = "MFINAV_WEBOTS_METHOD"
GOAL_TOLERANCE_ENV = "MFINAV_WEBOTS_GOAL_TOLERANCE"
CONFIG_JSON_ENV = "MFINAV_WEBOTS_CONFIG_JSON"

OBSTACLE_SPECS = [
    {"def": "OBSTACLE_A", "size": np.array([2.0, 2.0], dtype=float)},
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


def _load_env_scenario() -> dict[str, object] | None:
    raw = os.environ.get(SCENARIO_JSON_ENV)
    if not raw:
        return None
    return json.loads(raw)


def _apply_env_config_overrides(config) -> None:
    raw = os.environ.get(CONFIG_JSON_ENV)
    if not raw:
        return
    overrides = json.loads(raw)
    for key, value in overrides.items():
        if hasattr(config, key):
            setattr(config, key, value)


def _obstacle_from_spec(spec: dict[str, object]):
    kind = str(spec.get("kind", "polygon"))
    if kind == "circle":
        center = np.asarray(spec["center"], dtype=float)
        radius = float(spec["radius"])
        velocity = np.asarray(spec.get("velocity", [0.0, 0.0]), dtype=float)
        oscillation_amplitude = spec.get("oscillation_amplitude")
        oscillation_frequency = spec.get("oscillation_frequency")
        oscillation_phase = spec.get("oscillation_phase")
        is_dynamic = (
            float(np.linalg.norm(velocity)) > 0.0
            or oscillation_amplitude is not None
            or oscillation_frequency is not None
        )
        if is_dynamic:
            return MovingCircleObstacle(
                initial_center=center,
                radius=radius,
                velocity=velocity,
                oscillation_amplitude=None if oscillation_amplitude is None else np.asarray(oscillation_amplitude, dtype=float),
                oscillation_frequency=None if oscillation_frequency is None else np.asarray(oscillation_frequency, dtype=float),
                oscillation_phase=None if oscillation_phase is None else np.asarray(oscillation_phase, dtype=float),
            )
        return CircleObstacle(center=center, radius=radius)

    if kind == "polygon":
        if "initial_vertices" in spec:
            initial_vertices = np.asarray(spec["initial_vertices"], dtype=float)
        else:
            initial_vertices = np.asarray(spec["vertices"], dtype=float)
        velocity = np.asarray(spec.get("velocity", [0.0, 0.0]), dtype=float)
        oscillation_amplitude = spec.get("oscillation_amplitude")
        oscillation_frequency = spec.get("oscillation_frequency")
        oscillation_phase = spec.get("oscillation_phase")
        is_dynamic = (
            float(np.linalg.norm(velocity)) > 0.0
            or oscillation_amplitude is not None
            or oscillation_frequency is not None
        )
        if is_dynamic:
            return MovingPolygonObstacle(
                initial_vertices=initial_vertices,
                velocity=velocity,
                oscillation_amplitude=None if oscillation_amplitude is None else np.asarray(oscillation_amplitude, dtype=float),
                oscillation_frequency=None if oscillation_frequency is None else np.asarray(oscillation_frequency, dtype=float),
                oscillation_phase=None if oscillation_phase is None else np.asarray(oscillation_phase, dtype=float),
            )
        return PolygonObstacle(vertices=initial_vertices)

    raise ValueError(f"Unsupported scenario obstacle kind: {kind}")


def _update_visual_node_from_snapshot(node, snapshot: dict[str, object]) -> None:
    translation_field = node.getField("translation")
    if translation_field is None:
        return
    kind = snapshot.get("kind")
    if kind == "circle":
        center = snapshot["center"]
        translation_field.setSFVec3f([float(center[0]), float(center[1]), 0.06])
        return
    if kind == "polygon":
        vertices = np.asarray(snapshot["vertices"], dtype=float)
        centroid = np.mean(vertices, axis=0)
        translation_field.setSFVec3f([float(centroid[0]), float(centroid[1]), 0.0])
        return


def main() -> None:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0 if timestep > 0 else DEFAULT_DT
    max_steps = int(os.environ.get("MFINAV_WEBOTS_MAX_STEPS", str(DEFAULT_MAX_STEPS)))
    history_output = os.environ.get("MFINAV_WEBOTS_HISTORY")
    summary_output = os.environ.get("MFINAV_WEBOTS_SUMMARY")
    auto_quit = os.environ.get("MFINAV_WEBOTS_QUIT", "0") == "1"
    internal_dt = float(os.environ.get("MFINAV_WEBOTS_INTERNAL_DT", str(DEFAULT_INTERNAL_DT)))
    goal_tolerance = float(os.environ.get(GOAL_TOLERANCE_ENV, str(GOAL_TOLERANCE)))
    method_name = os.environ.get(METHOD_ENV, "paper_geometric")

    if method_name == "paper_pd":
        config = make_paper_pd_config()
        navigator_factory = lambda cfg: ReferenceNavigator(cfg)
    elif method_name == "paper_geometric":
        config = make_paper_geometric_config()
        navigator_factory = lambda cfg: ReferenceNavigator(cfg)
    elif method_name == "apf":
        config = make_paper_pd_config()
        navigator_factory = lambda cfg: ArtificialPotentialFieldNavigator(cfg)
    elif method_name == "haddadin":
        config = make_paper_pd_config()
        navigator_factory = lambda cfg: HaddadinNavigator(cfg)
    elif method_name == "sabattini":
        config = make_paper_pd_config()
        navigator_factory = lambda cfg: SabattiniNavigator(cfg)
    else:
        raise ValueError(f"Unsupported Webots method: {method_name}")

    config.max_linear_speed = 0.18
    config.max_angular_speed = 4.0
    config.speed_gain = 1.2
    config.heading_gain = 3.5
    config.min_forward_factor = 0.2
    _apply_env_config_overrides(config)

    navigator = navigator_factory(config)
    model = DifferentialDriveModel(config)
    sensing = LocalSensingModel(config)

    self_node = robot.getSelf()
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")
    goal_node = robot.getFromDef("GOAL")
    scenario_data = _load_env_scenario()
    obstacle_nodes = []
    scenario_obstacles: list[PolygonObstacle] | None = None
    scenario_visual_nodes: list[tuple[object, object]] = []
    scenario_name = "manual_world"

    if scenario_data is not None:
        scenario_name = str(scenario_data.get("name", scenario_name))
        start = np.asarray(scenario_data["start"], dtype=float)
        goal = np.asarray(scenario_data["goal"], dtype=float)
        heading = float(
            scenario_data.get(
                "heading",
                math.atan2(float(goal[1] - start[1]), float(goal[0] - start[0])),
            )
        )
        scenario_obstacles = []
        for index, obstacle_spec in enumerate(scenario_data["obstacles"]):
            obstacle_object = _obstacle_from_spec(obstacle_spec)
            scenario_obstacles.append(obstacle_object)
            obstacle_def = obstacle_spec.get("def", f"OBSTACLE_{index + 1}")
            visual_node = robot.getFromDef(str(obstacle_def))
            if visual_node is not None:
                scenario_visual_nodes.append((visual_node, obstacle_object))
        translation_field.setSFVec3f([float(start[0]), float(start[1]), 0.035])
        rotation_field.setSFRotation([0.0, 0.0, 1.0, heading])
        if goal_node is not None:
            goal_field = goal_node.getField("translation")
            if goal_field is not None:
                goal_field.setSFVec3f([float(goal[0]), float(goal[1]), 0.02])
        state = DifferentialDriveState(
            position=start.copy(),
            heading=heading,
            linear_speed=0.0,
            angular_speed=0.0,
        )
        initial_position = np.array([float(start[0]), float(start[1]), 0.035], dtype=float)
    else:
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
        "goal_tolerance": goal_tolerance,
        "scenario": scenario_name,
        "method": method_name,
    }
    exit_code = 0

    for step in range(max_steps):
        if robot.step(timestep) == -1:
            summary["status"] = "webots_stopped"
            break
        time_s = step * dt
        if scenario_data is not None:
            goal_position = np.asarray(scenario_data["goal"], dtype=float)
            obstacles = scenario_obstacles or []
            for obstacle in obstacles:
                if hasattr(obstacle, "set_time"):
                    obstacle.set_time(time_s)
            for visual_node, obstacle in scenario_visual_nodes:
                if hasattr(obstacle, "snapshot"):
                    _update_visual_node_from_snapshot(visual_node, obstacle.snapshot())
        else:
            goal_position = np.asarray(goal_node.getPosition(), dtype=float)[:2]
            obstacles = [_obstacle_from_node(node, size_xy) for node, size_xy in obstacle_nodes]
        if not obstacles:
            continue
        obstacle_collection = ObstacleCollection(obstacles)

        guidance = np.asarray(navigator.command(state, goal_position, obstacle_collection), dtype=float)
        observation = sensing.observe(state, obstacle_collection)
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
            guidance_step = np.asarray(navigator.command(state, goal_position, obstacle_collection), dtype=float)
            command = model.guidance_to_command(state, guidance_step)
            state = model.step(state, command, step_dt)
            remaining_dt -= step_dt
        translation_field.setSFVec3f([float(state.position[0]), float(state.position[1]), 0.035])
        rotation_field.setSFRotation([0.0, 0.0, 1.0, float(state.heading)])

        summary["steps"] = step + 1

        if closest_clearance <= 0.0:
            summary["status"] = "collision"
            exit_code = 2
            print("Collision detected in Webots arena.")
            break

        if goal_distance <= goal_tolerance:
            summary["status"] = "goal_reached"
            print("Goal reached in Webots arena.")
            break
    else:
        summary["status"] = "timeout"
        exit_code = 1
        print("Webots arena run reached the step limit without reaching the goal.")

    if history:
        metrics = compute_metrics(history, goal_position, success_radius=goal_tolerance)
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
