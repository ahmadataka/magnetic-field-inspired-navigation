from __future__ import annotations

import csv
import json
import os
from pathlib import Path

from controller import Supervisor


HISTORY_ENV = "MFINAV_WEBOTS_HISTORY"
SUMMARY_ENV = "MFINAV_WEBOTS_SUMMARY"
AUTO_QUIT_ENV = "MFINAV_WEBOTS_QUIT"
MAX_TIME_ENV = "MFINAV_WEBOTS_MAX_TIME"


def _write_history(rows: list[dict[str, float]], path: Path) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    history_path = os.environ.get(HISTORY_ENV)
    summary_path = os.environ.get(SUMMARY_ENV)
    auto_quit = os.environ.get(AUTO_QUIT_ENV, "0") == "1"
    max_time = float(os.environ.get(MAX_TIME_ENV, "10.0"))

    node = robot.getFromDef("CF")
    if node is None:
        raise RuntimeError("Missing DEF CF in stock Crazyflie validation world.")

    rows: list[dict[str, float]] = []
    max_z = float("-inf")

    status = "timeout"
    exit_code = 1
    while robot.step(timestep) != -1:
        now = robot.getTime()
        pos = node.getPosition()
        rows.append({"time": float(now), "x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])})
        max_z = max(max_z, float(pos[2]))
        if now >= max_time:
            status = "finished"
            exit_code = 0
            break

    summary = {
        "status": status,
        "max_z": max_z,
        "final_state": rows[-1] if rows else None,
    }
    if history_path:
        _write_history(rows, Path(history_path))
    if summary_path:
        Path(summary_path).write_text(json.dumps(summary, indent=2))
    if auto_quit:
        robot.simulationQuit(exit_code)
