#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import csv
from dataclasses import replace
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PolygonPatch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mfinav import (
    ArtificialPotentialFieldNavigator,
    DifferentialDriveState,
    HaddadinNavigator,
    PolygonObstacle,
    ReferenceNavigator,
    SabattiniNavigator,
    compute_metrics,
    make_default_scenarios,
    make_paper_geometric_config,
    make_paper_pd_config,
    simulate_differential_drive,
)


COLORS = {
    "paper_pd": "#1f77b4",
    "paper_geometric": "#2ca02c",
    "apf": "#ff7f0e",
    "haddadin": "#8b5cf6",
    "sabattini": "#d97706",
}


def _initial_state(start: np.ndarray, goal: np.ndarray) -> DifferentialDriveState:
    goal_vector = goal - start
    heading = math.atan2(float(goal_vector[1]), float(goal_vector[0]))
    return DifferentialDriveState(position=start.copy(), heading=heading)


def _diff_drive_config(base_config):
    return replace(
        base_config,
        max_linear_speed=1.2,
        max_angular_speed=3.0,
        speed_gain=1.5,
        heading_gain=4.0,
        min_forward_factor=0.25,
    )


def _plot_scenario(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]]) -> None:
    for name, history in histories.items():
        xs = [row["x"] for row in history]
        ys = [row["y"] for row in history]
        ax.plot(xs, ys, linewidth=2.0, label=name.upper(), color=COLORS[name])
        ax.scatter(xs[-1], ys[-1], color=COLORS[name], s=55, marker="x")

    for idx, obstacle in enumerate(scenario.obstacles.obstacles):
        if isinstance(obstacle, PolygonObstacle):
            patch = PolygonPatch(obstacle.vertices, closed=True, color="#d62728", alpha=0.18)
            centroid = np.mean(obstacle.vertices, axis=0)
            anchor = (float(centroid[0]), float(centroid[1]))
            ax.add_patch(patch)
            if idx == 0:
                ax.annotate("obstacles", anchor, textcoords="offset points", xytext=(6, 6))

    ax.scatter(scenario.start[0], scenario.start[1], color="#2ca02c", s=70, marker="o", label="start")
    ax.scatter(scenario.goal[0], scenario.goal[1], color="#111111", s=90, marker="*", label="goal")
    ax.set_title(f"{scenario.name.replace('_', ' ')} (diff-drive)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)


def main() -> None:
    scenarios = make_default_scenarios()
    config_pd = _diff_drive_config(make_paper_pd_config())
    config_geometric = _diff_drive_config(make_paper_geometric_config())
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(parents=True, exist_ok=True)

    summary_rows: list[dict[str, str | float]] = []
    cols = min(3, len(scenarios))
    rows = math.ceil(len(scenarios) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 5.2 * rows))
    axes = np.atleast_1d(axes).ravel()

    for ax, scenario in zip(axes, scenarios):
        histories = {
            "paper_pd": simulate_differential_drive(
                _initial_state(scenario.start, scenario.goal),
                scenario.goal,
                scenario.obstacles,
                config_pd,
                navigator=ReferenceNavigator(config_pd),
            ),
            "paper_geometric": simulate_differential_drive(
                _initial_state(scenario.start, scenario.goal),
                scenario.goal,
                scenario.obstacles,
                config_geometric,
                navigator=ReferenceNavigator(config_geometric),
            ),
            "apf": simulate_differential_drive(
                _initial_state(scenario.start, scenario.goal),
                scenario.goal,
                scenario.obstacles,
                config_pd,
                navigator=ArtificialPotentialFieldNavigator(config_pd),
            ),
            "haddadin": simulate_differential_drive(
                _initial_state(scenario.start, scenario.goal),
                scenario.goal,
                scenario.obstacles,
                config_pd,
                navigator=HaddadinNavigator(config_pd),
            ),
            "sabattini": simulate_differential_drive(
                _initial_state(scenario.start, scenario.goal),
                scenario.goal,
                scenario.obstacles,
                config_pd,
                navigator=SabattiniNavigator(config_pd),
            ),
        }

        _plot_scenario(ax, scenario, histories)
        for method_name, history in histories.items():
            metrics = compute_metrics(history, scenario.goal)
            summary_rows.append(
                {
                    "scenario": scenario.name,
                    "method": method_name,
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

    for ax in axes[len(scenarios):]:
        ax.axis("off")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=5, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    plot_path = artifacts / "benchmark_comparison_diff_drive.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    csv_path = artifacts / "benchmark_metrics_diff_drive.csv"
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

    print(f"benchmark_plot_diff_drive={plot_path}")
    print(f"benchmark_metrics_diff_drive={csv_path}")


if __name__ == "__main__":
    main()
