#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import csv
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
    DoubleIntegratorState,
    MagneticFieldNavigator3D,
    SphereObstacle,
    compute_metrics,
    make_default_scenarios_3d,
    make_paper_pd_config,
    simulate,
)


def _initial_state(start: np.ndarray) -> DoubleIntegratorState:
    return DoubleIntegratorState(position=start.copy(), velocity=np.zeros_like(start))


def _plot_projection(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]], axes: tuple[str, str]) -> None:
    colors = {
        "paper_pd_3d": "#1f77b4",
    }
    x_key, y_key = axes
    for name, history in histories.items():
        xs = [row[x_key] for row in history]
        ys = [row[y_key] for row in history]
        ax.plot(xs, ys, linewidth=2.0, label=name.upper(), color=colors[name])
        ax.scatter(xs[-1], ys[-1], color=colors[name], s=45, marker="x")

    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, SphereObstacle):
            center = obstacle.center
            ax.scatter(center[0 if x_key == "x" else 1 if x_key == "y" else 2], center[0 if y_key == "x" else 1 if y_key == "y" else 2], color="#d62728", s=80, alpha=0.35)

    ax.scatter(
        scenario.start[0 if x_key == "x" else 1 if x_key == "y" else 2],
        scenario.start[0 if y_key == "x" else 1 if y_key == "y" else 2],
        color="#2ca02c",
        s=60,
        marker="o",
    )
    ax.scatter(
        scenario.goal[0 if x_key == "x" else 1 if x_key == "y" else 2],
        scenario.goal[0 if y_key == "x" else 1 if y_key == "y" else 2],
        color="#111111",
        s=90,
        marker="*",
    )
    ax.set_xlabel(x_key)
    ax.set_ylabel(y_key)
    ax.grid(True, alpha=0.25)


def main() -> None:
    scenarios = make_default_scenarios_3d()
    config = make_paper_pd_config()
    config.sensing_mode = "analytic"
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(parents=True, exist_ok=True)

    summary_rows: list[dict[str, str | float]] = []
    fig, axes = plt.subplots(len(scenarios), 3, figsize=(16, 4.8 * len(scenarios)))
    axes = np.atleast_2d(axes)

    for row_axes, scenario in zip(axes, scenarios):
        history = simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            config,
            navigator=MagneticFieldNavigator3D(config),
        )
        histories = {"paper_pd_3d": history}
        for ax, proj in zip(row_axes, (("x", "y"), ("x", "z"), ("y", "z"))):
            _plot_projection(ax, scenario, histories, proj)
            ax.set_title(f"{scenario.name} {proj[0]}{proj[1]}")

        metrics = compute_metrics(history, scenario.goal)
        summary_rows.append(
            {
                "scenario": scenario.name,
                "method": "paper_pd_3d",
                "success": metrics["success"],
                "goal_reached_once": metrics["goal_reached_once"],
                "steps": metrics["steps"],
                "path_length": metrics["path_length"],
                "final_goal_distance": metrics["final_goal_distance"],
                "min_clearance": metrics["min_clearance"],
                "mean_speed": metrics["mean_speed"],
                "collision": metrics["collision"],
                "safety_violation": metrics["safety_violation"],
                "time_to_goal_steps": metrics["time_to_goal_steps"],
                "path_efficiency": metrics["path_efficiency"],
            }
        )

    fig.tight_layout()
    plot_path = artifacts / "benchmark_comparison_3d.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    csv_path = artifacts / "benchmark_metrics_3d.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "scenario",
                "method",
                "success",
                "goal_reached_once",
                "steps",
                "path_length",
                "final_goal_distance",
                "min_clearance",
                "mean_speed",
                "collision",
                "safety_violation",
                "time_to_goal_steps",
                "path_efficiency",
            ],
        )
        writer.writeheader()
        writer.writerows(summary_rows)

    for row in summary_rows:
        print(
            f"{row['scenario']} {row['method']}: "
            f"success={int(row['success'])} "
            f"goal_reached_once={int(row['goal_reached_once'])} "
            f"collision={int(row['collision'])} "
            f"safety_violation={int(row['safety_violation'])} "
            f"final_goal_distance={row['final_goal_distance']:.3f} "
            f"min_clearance={row['min_clearance']:.3f} "
            f"path_length={row['path_length']:.3f} "
            f"time_to_goal_steps={row['time_to_goal_steps'] if row['time_to_goal_steps'] != float('inf') else 'inf'}"
        )

    print(f"benchmark_plot_3d={plot_path}")
    print(f"benchmark_metrics_3d={csv_path}")


if __name__ == "__main__":
    main()
