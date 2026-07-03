#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PolygonPatch

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mfinav import PolygonObstacle  # noqa: E402


WORLD = ROOT / "webots" / "worlds" / "mfi_epuck_arena.wbt"
ARTIFACTS = ROOT / "artifacts"
HISTORY_CSV = ARTIFACTS / "webots_epuck_history.csv"
SUMMARY_JSON = ARTIFACTS / "webots_epuck_summary.json"
PLOT_PNG = ARTIFACTS / "webots_epuck_trajectory.png"
CONSOLE_LOG = ARTIFACTS / "webots_epuck_console.log"

COLORS = {
    "trajectory": "#2ca02c",
    "obstacle": "#d62728",
    "start": "#2ca02c",
    "goal": "#111111",
}

OBSTACLE_SPECS = [
    {"center": (0.25, 0.15), "size": (0.22, 0.22), "yaw": 0.3},
    {"center": (0.65, 0.7), "size": (0.26, 0.18), "yaw": -0.25},
    {"center": (0.8, -0.55), "size": (0.18, 0.28), "yaw": 0.6},
]
START = (-1.1, -1.0)
GOAL = (1.1, 1.0)


def _rotation_matrix_2d(theta: float) -> list[list[float]]:
    import math
    return [
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)],
    ]


def _polygon_from_box(center: tuple[float, float], size: tuple[float, float], yaw: float) -> PolygonObstacle:
    import numpy as np

    cx, cy = center
    sx, sy = size
    half_x = 0.5 * sx
    half_y = 0.5 * sy
    corners = np.array(
        [
            [-half_x, -half_y],
            [half_x, -half_y],
            [half_x, half_y],
            [-half_x, half_y],
        ],
        dtype=float,
    )
    rotation = np.asarray(_rotation_matrix_2d(yaw), dtype=float)
    vertices = (corners @ rotation.T) + np.array([cx, cy], dtype=float)
    return PolygonObstacle(vertices=vertices)


def _load_history(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return [{key: float(value) for key, value in row.items()} for row in rows]


def _plot(history: list[dict[str, float]], summary: dict[str, object]) -> None:
    fig, ax = plt.subplots(figsize=(8.2, 6.4))
    xs = [row["x"] for row in history]
    ys = [row["y"] for row in history]
    ax.plot(xs, ys, linewidth=2.2, color=COLORS["trajectory"], label="Webots MFI")
    ax.scatter(xs[-1], ys[-1], color=COLORS["trajectory"], s=60, marker="x", label="final")

    for index, spec in enumerate(OBSTACLE_SPECS):
        polygon = _polygon_from_box(spec["center"], spec["size"], spec["yaw"])
        patch = PolygonPatch(polygon.vertices, closed=True, color=COLORS["obstacle"], alpha=0.18)
        ax.add_patch(patch)
        if index == 0:
            centroid = polygon.vertices.mean(axis=0)
            ax.annotate("obstacles", (float(centroid[0]), float(centroid[1])), textcoords="offset points", xytext=(6, 6))

    ax.scatter(START[0], START[1], color=COLORS["start"], s=70, marker="o", label="start")
    ax.scatter(GOAL[0], GOAL[1], color=COLORS["goal"], s=90, marker="*", label="goal")

    status = summary.get("status", "unknown")
    metrics = summary.get("metrics", {})
    final_error = metrics.get("final_goal_distance", float("nan")) if isinstance(metrics, dict) else float("nan")
    min_clearance = metrics.get("min_clearance", float("nan")) if isinstance(metrics, dict) else float("nan")
    ax.set_title(f"Webots e-puck evaluation | status={status} | final error={final_error:.3f} | min clearance={min_clearance:.3f}")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(frameon=False, loc="upper left")
    fig.tight_layout()
    fig.savefig(PLOT_PNG, dpi=180)
    plt.close(fig)


def main() -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["MFINAV_WEBOTS_HISTORY"] = str(HISTORY_CSV)
    env["MFINAV_WEBOTS_SUMMARY"] = str(SUMMARY_JSON)
    env["MFINAV_WEBOTS_MAX_STEPS"] = env.get("MFINAV_WEBOTS_MAX_STEPS", "2500")
    env["MFINAV_WEBOTS_QUIT"] = "1"

    command = [
        "/Applications/Webots.app/Contents/MacOS/webots",
        "--batch",
        "--mode=fast",
        "--no-rendering",
        "--stdout",
        "--stderr",
        str(WORLD),
    ]

    result = subprocess.run(
        command,
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    CONSOLE_LOG.write_text(result.stdout)

    if not HISTORY_CSV.exists():
        raise SystemExit(
            "Webots finished without producing a trajectory log. "
            f"See {CONSOLE_LOG} for details."
        )

    history = _load_history(HISTORY_CSV)
    summary = json.loads(SUMMARY_JSON.read_text()) if SUMMARY_JSON.exists() else {"status": "unknown"}
    _plot(history, summary)

    print(f"webots_console={CONSOLE_LOG}")
    print(f"webots_history={HISTORY_CSV}")
    print(f"webots_summary={SUMMARY_JSON}")
    print(f"webots_plot={PLOT_PNG}")
    print(f"webots_exit_code={result.returncode}")
    print(f"webots_status={summary.get('status', 'unknown')}")


if __name__ == "__main__":
    main()
