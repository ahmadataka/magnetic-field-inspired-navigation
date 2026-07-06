#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
ARTIFACTS = ROOT / "artifacts"
WORLD_PATH = ROOT / "webots" / "worlds" / "mfi_crazyflie_arena.wbt"
HISTORY_PATH = ARTIFACTS / "webots_crazyflie_history.csv"
SUMMARY_PATH = ARTIFACTS / "webots_crazyflie_summary.json"
CONSOLE_PATH = ARTIFACTS / "webots_crazyflie_console.log"
PLOT_PATH = ARTIFACTS / "webots_crazyflie_smoke.png"


def _load_history(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return [{key: float(value) for key, value in row.items()} for row in rows]


def _plot(history: list[dict[str, float]], summary: dict[str, object]) -> None:
    xs = [row["x"] for row in history]
    ys = [row["y"] for row in history]
    zs = [row["z"] for row in history]
    start = summary["start"]
    goal = summary["goal"]

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.6))

    ax = axes[0]
    ax.plot(xs, ys, color="#2563eb", linewidth=2.2)
    ax.scatter([start[0]], [start[1]], color="#16a34a", s=55, marker="o", label="start")
    ax.scatter([goal[0]], [goal[1]], color="#111111", s=90, marker="*", label="goal")
    ax.add_patch(plt.Rectangle((0.4 - 0.175, 0.1 - 0.175), 0.35, 0.35, color="#ef4444", alpha=0.22))
    ax.add_patch(plt.Rectangle((-0.3 - 0.125, 0.85 - 0.275), 0.25, 0.55, color="#ef4444", alpha=0.22))
    ax.set_title("Top view")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")

    ax = axes[1]
    ax.plot(range(len(zs)), zs, color="#2563eb", linewidth=2.2)
    ax.axhline(goal[2], color="#111111", linestyle="--", linewidth=1.5, label="goal z")
    ax.set_title("Altitude")
    ax.set_xlabel("step")
    ax.set_ylabel("z")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")

    fig.suptitle(f"Webots Crazyflie smoke test ({summary['status']})")
    fig.tight_layout()
    fig.savefig(PLOT_PATH, dpi=180)
    plt.close(fig)


def main() -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["MFINAV_WEBOTS_HISTORY"] = str(HISTORY_PATH)
    env["MFINAV_WEBOTS_SUMMARY"] = str(SUMMARY_PATH)
    env["MFINAV_WEBOTS_QUIT"] = "1"
    env["MFINAV_WEBOTS_MAX_STEPS"] = env.get("MFINAV_WEBOTS_MAX_STEPS", "1800")

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
    CONSOLE_PATH.write_text(result.stdout)
    if not HISTORY_PATH.exists():
        raise RuntimeError(f"Webots Crazyflie run did not produce a history. See {CONSOLE_PATH}.")

    history = _load_history(HISTORY_PATH)
    summary = json.loads(SUMMARY_PATH.read_text())
    summary["webots_exit_code"] = result.returncode
    SUMMARY_PATH.write_text(json.dumps(summary, indent=2))
    _plot(history, summary)

    print(
        f"status={summary['status']} "
        f"steps={summary['steps']} "
        f"final_goal_distance={summary['final_goal_distance']:.3f}"
    )
    print(f"webots_crazyflie_plot={PLOT_PATH}")
    print(f"webots_crazyflie_history={HISTORY_PATH}")
    print(f"webots_crazyflie_summary={SUMMARY_PATH}")


if __name__ == "__main__":
    main()
