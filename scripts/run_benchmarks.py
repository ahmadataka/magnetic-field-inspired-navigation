#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import csv
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
    CircleObstacle,
    DoubleIntegratorState,
    PolygonObstacle,
    ReferenceNavigator,
    compute_metrics,
    make_default_scenarios,
    make_paper_geometric_config,
    make_paper_pd_config,
    make_pragmatic_config,
    simulate,
)


def _initial_state(start: np.ndarray) -> DoubleIntegratorState:
    return DoubleIntegratorState(position=start.copy(), velocity=np.array([0.0, 0.0], dtype=float))


def _plot_scenario(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]]) -> None:
    colors = {
        "paper_pd": "#1f77b4",
        "paper_geometric": "#9467bd",
        "pragmatic_mfi": "#2ca02c",
        "apf": "#ff7f0e",
    }
    for name, history in histories.items():
        xs = [row["x"] for row in history]
        ys = [row["y"] for row in history]
        ax.plot(xs, ys, linewidth=2.0, label=name.upper(), color=colors[name])
        ax.scatter(xs[-1], ys[-1], color=colors[name], s=55, marker="x")

    for idx, obstacle in enumerate(scenario.obstacles.obstacles):
        if isinstance(obstacle, CircleObstacle):
            patch = plt.Circle(tuple(obstacle.center), obstacle.radius, color="#d62728", alpha=0.18)
            anchor = tuple(obstacle.center)
        elif isinstance(obstacle, PolygonObstacle):
            patch = PolygonPatch(obstacle.vertices, closed=True, color="#d62728", alpha=0.18)
            centroid = np.mean(obstacle.vertices, axis=0)
            anchor = (float(centroid[0]), float(centroid[1]))
        else:
            continue
        ax.add_patch(patch)
        if idx == 0:
            ax.annotate("obstacles", anchor, textcoords="offset points", xytext=(6, 6))

    ax.scatter(scenario.start[0], scenario.start[1], color="#2ca02c", s=70, marker="o", label="start")
    ax.scatter(scenario.goal[0], scenario.goal[1], color="#111111", s=90, marker="*", label="goal")
    ax.set_title(scenario.name.replace("_", " "))
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)


def main() -> None:
    scenarios = make_default_scenarios()
    paper_pd_config = make_paper_pd_config()
    paper_geometric_config = make_paper_geometric_config()
    pragmatic_config = make_pragmatic_config()
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(parents=True, exist_ok=True)

    summary_rows: list[dict[str, str | float]] = []
    fig, axes = plt.subplots(1, len(scenarios), figsize=(6 * len(scenarios), 5.5))
    if len(scenarios) == 1:
        axes = [axes]

    for ax, scenario in zip(axes, scenarios):
        mfi_history = simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            paper_pd_config,
            navigator=ReferenceNavigator(paper_pd_config),
        )
        paper_geometric_history = simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            paper_geometric_config,
            navigator=ReferenceNavigator(paper_geometric_config),
        )
        pragmatic_history = simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            pragmatic_config,
            navigator=ReferenceNavigator(pragmatic_config),
        )
        apf_history = simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            pragmatic_config,
            navigator=ArtificialPotentialFieldNavigator(pragmatic_config),
        )

        histories = {
            "paper_pd": mfi_history,
            "paper_geometric": paper_geometric_history,
            "pragmatic_mfi": pragmatic_history,
            "apf": apf_history,
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

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    plot_path = artifacts / "benchmark_comparison.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    csv_path = artifacts / "benchmark_metrics.csv"
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

    print(f"benchmark_plot={plot_path}")
    print(f"benchmark_metrics={csv_path}")


if __name__ == "__main__":
    main()
