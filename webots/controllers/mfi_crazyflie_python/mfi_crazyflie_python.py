from __future__ import annotations

import csv
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from controller import Supervisor

from pid_controller import pid_velocity_fixed_height_controller


PROJECT_ROOT = Path(__file__).resolve().parents[3]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from mfinav import (
    DoubleIntegratorState,
    MagneticFieldNavigator,
    ObstacleCollection,
    PolygonObstacle,
    compute_metrics,
    make_paper_geometric_config,
    make_paper_pd_config,
)

HISTORY_ENV = "MFINAV_WEBOTS_HISTORY"
SUMMARY_ENV = "MFINAV_WEBOTS_SUMMARY"
AUTO_QUIT_ENV = "MFINAV_WEBOTS_QUIT"
MAX_STEPS_ENV = "MFINAV_WEBOTS_MAX_STEPS"
NAV_MODE_ENV = "MFINAV_WEBOTS_NAV_MODE"
SCENARIO_JSON_ENV = "MFINAV_WEBOTS_SCENARIO_JSON"
METHOD_ENV = "MFINAV_WEBOTS_METHOD"
CONFIG_JSON_ENV = "MFINAV_WEBOTS_CONFIG_JSON"

GOAL = np.array([1.2, 0.9, 1.0], dtype=float)
GOAL_TOLERANCE = 0.18
ALTITUDE_TOLERANCE = 0.12
VELOCITY_TOLERANCE = 0.12
XY_SPEED_LIMIT = 0.35
YAW_RATE_LIMIT = 1.2
KP_XY = 0.8
KP_YAW = 1.4
DEFAULT_MAX_STEPS = 1800
HOVER_HEIGHT = 1.0
MIN_COLLISION_STEP = 500
MOVE_ENABLE_TIME = 6.0
MFI_OBSTACLES = ObstacleCollection(
    obstacles=[
        PolygonObstacle(vertices=np.array([[0.225, -0.075], [0.575, -0.075], [0.575, 0.275], [0.225, 0.275]], dtype=float)),
        PolygonObstacle(vertices=np.array([[-0.425, 0.575], [-0.175, 0.575], [-0.175, 1.125], [-0.425, 1.125]], dtype=float)),
    ]
)


def _load_scenario() -> tuple[np.ndarray, np.ndarray, float, ObstacleCollection, str]:
    raw = os.environ.get(SCENARIO_JSON_ENV)
    if not raw:
        return GOAL.copy(), np.array([-1.2, -0.9, 0.08], dtype=float), HOVER_HEIGHT, MFI_OBSTACLES, "webots_crazyflie_static_smoke"
    data = json.loads(raw)
    goal = np.asarray(data["goal"], dtype=float)
    start = np.asarray(data["start"], dtype=float)
    hover_height = float(data.get("hover_height", HOVER_HEIGHT))
    obstacles = []
    for obstacle_spec in data.get("obstacles", []):
        obstacles.append(PolygonObstacle(vertices=np.asarray(obstacle_spec["vertices"], dtype=float)))
    return goal, start, hover_height, ObstacleCollection(obstacles=obstacles), str(data.get("name", "webots_crazyflie"))


def _apply_overrides(config) -> None:
    raw = os.environ.get(CONFIG_JSON_ENV)
    if not raw:
        return
    overrides = json.loads(raw)
    for key, value in overrides.items():
        if hasattr(config, key):
            setattr(config, key, value)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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


if __name__ == "__main__":
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    translation_field = robot.getSelf().getField("translation")

    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float("inf"))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float("inf"))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float("inf"))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float("inf"))
    m4_motor.setVelocity(1)

    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    pid = pid_velocity_fixed_height_controller()
    history_output = os.environ.get(HISTORY_ENV)
    summary_output = os.environ.get(SUMMARY_ENV)
    auto_quit = os.environ.get(AUTO_QUIT_ENV, "0") == "1"
    max_steps = int(os.environ.get(MAX_STEPS_ENV, str(DEFAULT_MAX_STEPS)))
    method_name = os.environ.get(METHOD_ENV, "")
    nav_mode = os.environ.get(NAV_MODE_ENV, "mfi" if method_name.startswith("paper_") else "waypoint")
    goal, start_translation, hover_height, mfi_obstacles, scenario_name = _load_scenario()

    if method_name == "paper_pd":
        mfi_config = make_paper_pd_config()
    else:
        mfi_config = make_paper_geometric_config()
    mfi_config.r_l = 0.9
    mfi_config.r_la = 0.45
    mfi_config.c_field = 2.8
    mfi_config.c_perp = 4.5
    mfi_config.speed_limit = XY_SPEED_LIMIT
    mfi_config.kp_goal_relaxed = 0.12
    mfi_config.kp_geom = 2.0
    mfi_config.sensor_range = 1.8
    mfi_config.delta_r = 0.12
    mfi_config.sensing_mode = "raycast"
    mfi_config.max_acceleration = 2.5
    mfi_config.max_speed_norm = XY_SPEED_LIMIT
    _apply_overrides(mfi_config)
    mfi_navigator = MagneticFieldNavigator(mfi_config)

    translation_field.setSFVec3f([float(start_translation[0]), float(start_translation[1]), float(start_translation[2])])

    while robot.step(timestep) != -1:
        if robot.getTime() > 2.0:
            break

    history: list[dict[str, float]] = []
    summary: dict[str, object] = {
        "status": "unknown",
        "scenario": scenario_name,
        "goal": goal.tolist(),
        "goal_tolerance": GOAL_TOLERANCE,
        "steps": 0,
        "nav_mode": nav_mode,
        "method": method_name or nav_mode,
    }

    past_position = np.array(gps.getValues(), dtype=float)
    past_time = robot.getTime()
    first_time = True
    exit_code = 0

    while robot.step(timestep) != -1:
        step = len(history)
        if step >= max_steps:
            summary["status"] = "timeout"
            exit_code = 1
            break

        now = robot.getTime()
        dt = max(now - past_time, 1e-3)
        position = np.array(gps.getValues(), dtype=float)
        roll, pitch, yaw = imu.getRollPitchYaw()
        yaw_rate = gyro.getValues()[2]

        if first_time:
            velocity_global = np.zeros(3, dtype=float)
            first_time = False
        else:
            velocity_global = (position - past_position) / dt

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        v_x = velocity_global[0] * cos_yaw + velocity_global[1] * sin_yaw
        v_y = -velocity_global[0] * sin_yaw + velocity_global[1] * cos_yaw

        goal_error_world = goal - position
        if now < MOVE_ENABLE_TIME:
            desired_vx = 0.0
            desired_vy = 0.0
            desired_yaw_rate = 0.0
            desired_altitude = float(hover_height)
        else:
            if nav_mode == "mfi":
                planar_state = DoubleIntegratorState(position=position[:2].copy(), velocity=velocity_global[:2].copy())
                planar_goal = goal[:2].copy()
                planar_guidance = np.asarray(mfi_navigator.command(planar_state, planar_goal, mfi_obstacles), dtype=float)
                desired_velocity_world = velocity_global[:2] + dt * planar_guidance
                speed_norm = float(np.linalg.norm(desired_velocity_world))
                if speed_norm > XY_SPEED_LIMIT:
                    desired_velocity_world *= XY_SPEED_LIMIT / speed_norm
            else:
                desired_velocity_world = KP_XY * goal_error_world[:2]
                speed_norm = float(np.linalg.norm(desired_velocity_world))
                if speed_norm > XY_SPEED_LIMIT:
                    desired_velocity_world *= XY_SPEED_LIMIT / speed_norm

            desired_vx = desired_velocity_world[0] * cos_yaw + desired_velocity_world[1] * sin_yaw
            desired_vy = -desired_velocity_world[0] * sin_yaw + desired_velocity_world[1] * cos_yaw

            desired_heading = math.atan2(goal_error_world[1], goal_error_world[0])
            yaw_error = _wrap_angle(desired_heading - yaw)
            desired_yaw_rate = max(-YAW_RATE_LIMIT, min(YAW_RATE_LIMIT, KP_YAW * yaw_error))
            desired_altitude = float(hover_height)

        motor_power = pid.pid(
            dt,
            desired_vx,
            desired_vy,
            desired_yaw_rate,
            desired_altitude,
            roll,
            pitch,
            yaw_rate,
            position[2],
            v_x,
            v_y,
        )

        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        goal_distance = float(np.linalg.norm(goal_error_world))
        signed_clearance = float(mfi_obstacles.clearance(position[:2])) if mfi_obstacles.obstacles else math.inf
        history.append(
            {
                "step": float(step),
                "time": float(now),
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
                "vx": float(velocity_global[0]),
                "vy": float(velocity_global[1]),
                "vz": float(velocity_global[2]),
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
                "yaw_rate": float(yaw_rate),
                "desired_vx_body": float(desired_vx),
                "desired_vy_body": float(desired_vy),
                "desired_yaw_rate": float(desired_yaw_rate),
                "desired_altitude": float(desired_altitude),
                "goal_distance": goal_distance,
                "signed_clearance": signed_clearance,
                "motor_m1": float(motor_power[0]),
                "motor_m2": float(motor_power[1]),
                "motor_m3": float(motor_power[2]),
                "motor_m4": float(motor_power[3]),
            }
        )

        summary["steps"] = step + 1
        if step >= MIN_COLLISION_STEP and position[2] < 0.04:
            summary["status"] = "collision"
            exit_code = 2
            break

        speed_total = float(np.linalg.norm(velocity_global))
        if (
            goal_distance <= GOAL_TOLERANCE
            and abs(position[2] - goal[2]) <= ALTITUDE_TOLERANCE
            and speed_total <= VELOCITY_TOLERANCE
        ):
            summary["status"] = "goal_reached"
            break

        past_time = now
        past_position = position.copy()

    final_position = np.array(gps.getValues(), dtype=float)
    summary["final_state"] = {
        "x": float(final_position[0]),
        "y": float(final_position[1]),
        "z": float(final_position[2]),
    }
    if history:
        summary["final_goal_distance"] = float(history[-1]["goal_distance"])
        summary["start"] = [history[0]["x"], history[0]["y"], history[0]["z"]]
        metrics = compute_metrics(history, goal, success_radius=GOAL_TOLERANCE)
        summary["metrics"] = {key: float(value) for key, value in metrics.items()}

    if history_output:
        _write_history_csv(history, Path(history_output))
    if summary_output:
        _write_summary_json(Path(summary_output), summary)

    if auto_quit:
        robot.simulationQuit(exit_code)
