from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path

import numpy as np
from controller import Supervisor

from pid_controller import pid_velocity_fixed_height_controller


PROJECT_ROOT = Path(__file__).resolve().parents[3]
HISTORY_ENV = "MFINAV_WEBOTS_HISTORY"
SUMMARY_ENV = "MFINAV_WEBOTS_SUMMARY"
AUTO_QUIT_ENV = "MFINAV_WEBOTS_QUIT"
MAX_STEPS_ENV = "MFINAV_WEBOTS_MAX_STEPS"

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

    while robot.step(timestep) != -1:
        if robot.getTime() > 2.0:
            break

    history: list[dict[str, float]] = []
    summary: dict[str, object] = {
        "status": "unknown",
        "scenario": "webots_crazyflie_static_smoke",
        "goal": GOAL.tolist(),
        "goal_tolerance": GOAL_TOLERANCE,
        "steps": 0,
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

        goal_error_world = GOAL - position
        if now < MOVE_ENABLE_TIME:
            desired_vx = 0.0
            desired_vy = 0.0
            desired_yaw_rate = 0.0
            desired_altitude = float(HOVER_HEIGHT)
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
            desired_altitude = float(HOVER_HEIGHT)

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
            and abs(position[2] - GOAL[2]) <= ALTITUDE_TOLERANCE
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

    if history_output:
        _write_history_csv(history, Path(history_output))
    if summary_output:
        _write_summary_json(Path(summary_output), summary)

    if auto_quit:
        robot.simulationQuit(exit_code)
