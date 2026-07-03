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
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


ARTIFACTS = ROOT / "artifacts"
WORLD_PATH = ROOT / "webots" / "worlds" / "mfi_quadrotor_arena.wbt"
PLOT_PATH = ARTIFACTS / "webots_quadrotor_smoke.png"
HISTORY_PATH = ARTIFACTS / "webots_quadrotor_history.csv"
SUMMARY_PATH = ARTIFACTS / "webots_quadrotor_summary.json"
CONSOLE_PATH = ARTIFACTS / "webots_quadrotor_console.log"
METHOD_ENV = "MFINAV_WEBOTS_METHOD"
CONFIG_JSON_ENV = "MFINAV_WEBOTS_CONFIG_JSON"
METHODS = ["paper_pd_3d", "paper_geometric_3d"]
METHOD_CONFIG_OVERRIDES = {
    "paper_pd_3d": {"kp_goal": 0.14, "kp_goal_relaxed": 0.14, "kd_goal": 0.5, "max_speed_norm": 1.8},
    "paper_geometric_3d": {},
}


def _load_history(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return [{key: float(value) for key, value in row.items()} for row in rows]


def _plot_obstacles(ax: plt.Axes) -> None:
    _ = ax

def _plot_histories(results: list[tuple[str, list[dict[str, float]], dict[str, object]]]) -> None:
    start = np.asarray(results[0][2]["start"], dtype=float)
    goal = np.asarray(results[0][2]["goal"], dtype=float)
    colors = {"paper_pd_3d": "#2563eb", "paper_geometric_3d": "#16a34a"}

    fig = plt.figure(figsize=(8.4, 6.4))
    ax = fig.add_subplot(111, projection="3d")
    _plot_obstacles(ax)
    for method_name, history, summary in results:
        xs = [row["x"] for row in history]
        ys = [row["y"] for row in history]
        zs = [row["z"] for row in history]
        label = f"{method_name} ({summary['status']})"
        ax.plot(xs, ys, zs, color=colors.get(method_name, "#2563eb"), linewidth=2.2, label=label)
        ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], color=colors.get(method_name, "#2563eb"), s=55, marker="x")
    ax.scatter([start[0]], [start[1]], [start[2]], color="#16a34a", s=70, marker="o", label="start")
    ax.scatter([goal[0]], [goal[1]], [goal[2]], color="#111111", s=110, marker="*", label="goal")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title("Webots quadrotor static MFI smoke test")
    ax.legend(loc="upper left")
    ax.view_init(elev=28, azim=-52)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOT_PATH, dpi=180)
    plt.close(fig)


def main() -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    requested_method = os.environ.get(METHOD_ENV)
    methods = [requested_method] if requested_method else METHODS
    results: list[tuple[str, list[dict[str, float]], dict[str, object]]] = []

    for method_name in methods:
        env = os.environ.copy()
        method_slug = method_name.lower()
        history_path = ARTIFACTS / f"webots_quadrotor_history_{method_slug}.csv"
        summary_path = ARTIFACTS / f"webots_quadrotor_summary_{method_slug}.json"
        console_path = ARTIFACTS / f"webots_quadrotor_console_{method_slug}.log"
        env["MFINAV_WEBOTS_HISTORY"] = str(history_path)
        env["MFINAV_WEBOTS_SUMMARY"] = str(summary_path)
        env["MFINAV_WEBOTS_QUIT"] = "1"
        env["MFINAV_WEBOTS_MAX_STEPS"] = env.get("MFINAV_WEBOTS_MAX_STEPS", "1800")
        env[METHOD_ENV] = method_name
        env[CONFIG_JSON_ENV] = json.dumps(METHOD_CONFIG_OVERRIDES.get(method_name, {}), separators=(",", ":"))

        command = [
            "/Applications/Webots.app/Contents/MacOS/webots",
            "--batch",
            "--mode=fast",
            "--no-rendering",
            "--stdout",
            "--stderr",
            str(WORLD_PATH),
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
        console_path.write_text(result.stdout)
        if not history_path.exists():
            raise RuntimeError(f"Webots quadrotor run did not produce a history for {method_name}. See {console_path}.")

        history = _load_history(history_path)
        summary = json.loads(summary_path.read_text())
        summary["webots_exit_code"] = result.returncode
        summary_path.write_text(json.dumps(summary, indent=2))
        results.append((method_name, history, summary))

        print(
            f"{method_name}: status={summary['status']} "
            f"steps={summary['steps']} "
            f"final_goal_distance={summary['metrics']['final_goal_distance']:.3f} "
            f"min_clearance={summary['metrics']['min_clearance']:.3f}"
        )

    if requested_method and results:
        method_name, history, summary = results[0]
        HISTORY_PATH.write_text((ARTIFACTS / f"webots_quadrotor_history_{method_name.lower()}.csv").read_text())
        SUMMARY_PATH.write_text(json.dumps(summary, indent=2))
        CONSOLE_PATH.write_text((ARTIFACTS / f"webots_quadrotor_console_{method_name.lower()}.log").read_text())

    _plot_histories(results)
    print(f"webots_quadrotor_plot={PLOT_PATH}")
    print(f"webots_quadrotor_history_dir={ARTIFACTS}")
    print(f"webots_quadrotor_summary_dir={ARTIFACTS}")


if __name__ == "__main__":
    main()
