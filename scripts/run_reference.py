#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mfinav import (
    CircleObstacle,
    DoubleIntegratorState,
    SimulationConfig,
    simulate,
)
from mfinav.double_integrator import write_history_csv


def main() -> None:
    state = DoubleIntegratorState(
        position=np.array([0.0, 0.0], dtype=float),
        velocity=np.array([0.0, 0.0], dtype=float),
    )
    goal = np.array([10.0, 1.5], dtype=float)
    obstacle = CircleObstacle(center=np.array([5.0, 0.0], dtype=float), radius=1.0)
    config = SimulationConfig()

    history = simulate(state, goal, obstacle, config)
    output = ROOT / "artifacts" / "reference_trajectory.csv"
    write_history_csv(history, output)
    plot_output = ROOT / "artifacts" / "reference_trajectory.png"

    xs = [row["x"] for row in history]
    ys = [row["y"] for row in history]
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(xs, ys, linewidth=2.0, color="#1f77b4", label="trajectory")

    obstacle_patch = plt.Circle(
        tuple(obstacle.center),
        obstacle.radius,
        color="#d62728",
        alpha=0.25,
        label="obstacle",
    )
    ax.add_patch(obstacle_patch)

    ax.scatter(xs[0], ys[0], color="#2ca02c", s=70, marker="o", label="start")
    ax.scatter(goal[0], goal[1], color="#ff7f0e", s=90, marker="*", label="goal")
    ax.scatter(xs[-1], ys[-1], color="#111111", s=70, marker="x", label="final")

    ax.annotate("start", (xs[0], ys[0]), textcoords="offset points", xytext=(8, 8))
    ax.annotate("goal", (goal[0], goal[1]), textcoords="offset points", xytext=(8, 8))
    ax.annotate("final", (xs[-1], ys[-1]), textcoords="offset points", xytext=(8, -14))
    ax.annotate("obstacle", tuple(obstacle.center), textcoords="offset points", xytext=(8, 8))

    ax.set_title("Magnetic-Field-Inspired Navigation Reference Trajectory")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(plot_output, dpi=180)
    plt.close(fig)

    final = history[-1]
    min_clearance = min(row["obstacle_distance"] for row in history)
    print(f"steps={len(history)}")
    print(f"final_goal_distance={final['goal_distance']:.4f}")
    print(f"min_obstacle_distance={min_clearance:.4f}")
    print(f"final_position=({final['x']:.4f}, {final['y']:.4f})")
    print(f"trajectory_csv={output}")
    print(f"trajectory_plot={plot_output}")


if __name__ == "__main__":
    main()
