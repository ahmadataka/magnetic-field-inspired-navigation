#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import csv
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
    DoubleIntegratorState,
    PolygonObstacle,
    ReferenceNavigator,
    compute_metrics,
    make_default_scenarios,
    make_paper_geometric_config,
    make_paper_pd_config,
)


def _initial_state(start: np.ndarray) -> DoubleIntegratorState:
    return DoubleIntegratorState(position=start.copy(), velocity=np.array([0.0, 0.0], dtype=float))


def _clip_norm(vec: np.ndarray, limit: float) -> np.ndarray:
    magnitude = float(np.linalg.norm(vec))
    if magnitude <= limit or magnitude < 1e-9:
        return vec
    return vec * (limit / magnitude)


def simulate_with_diagnostics(
    state: DoubleIntegratorState,
    goal: np.ndarray,
    obstacle,
    navigator: ReferenceNavigator,
) -> list[dict[str, float]]:
    cfg = navigator.goal_controller.config
    history: list[dict[str, float]] = []

    for step in range(cfg.steps):
        accel = navigator.command(state, goal, obstacle)
        accel = _clip_norm(accel, cfg.max_acceleration)
        observation = navigator.last_observation
        if observation is None:
            raise RuntimeError("Navigator diagnostics were not populated.")

        goal_cmd = navigator.last_goal_cmd
        boundary_cmd = navigator.last_boundary_cmd
        collision_cmd = navigator.last_collision_cmd
        total_cmd = navigator.last_total_cmd
        speed = float(np.linalg.norm(state.velocity))
        heading_error = 0.0
        goal_vector = goal - state.position
        goal_distance = float(np.linalg.norm(goal_vector))
        if speed > 1e-9 and goal_distance > 1e-9:
            vel_hat = state.velocity / speed
            goal_hat = goal_vector / goal_distance
            heading_error = math.atan2(
                float(vel_hat[0] * goal_hat[1] - vel_hat[1] * goal_hat[0]),
                float(np.dot(vel_hat, goal_hat)),
            )

        history.append(
            {
                "step": float(step),
                "x": float(state.position[0]),
                "y": float(state.position[1]),
                "vx": float(state.velocity[0]),
                "vy": float(state.velocity[1]),
                "speed": speed,
                "goal_distance": goal_distance,
                "obstacle_distance": float(observation.distance_to_obstacle),
                "signed_clearance": float(observation.signed_clearance),
                "closest_obstacle_x": float(observation.closest_obstacle_vector[0]),
                "closest_obstacle_y": float(observation.closest_obstacle_vector[1]),
                "used_obstacle_x": float(observation.used_obstacle_vector[0]),
                "used_obstacle_y": float(observation.used_obstacle_vector[1]),
                "avg_obstacle_x": float(observation.averaged_obstacle_vector[0]),
                "avg_obstacle_y": float(observation.averaged_obstacle_vector[1]),
                "goal_weight": float(navigator.goal_controller.last_goal_weight),
                "heading_error": heading_error,
                "goal_ax": float(goal_cmd[0]),
                "goal_ay": float(goal_cmd[1]),
                "goal_norm": float(np.linalg.norm(goal_cmd)),
                "boundary_ax": float(boundary_cmd[0]),
                "boundary_ay": float(boundary_cmd[1]),
                "boundary_norm": float(np.linalg.norm(boundary_cmd)),
                "collision_ax": float(collision_cmd[0]),
                "collision_ay": float(collision_cmd[1]),
                "collision_norm": float(np.linalg.norm(collision_cmd)),
                "total_ax": float(total_cmd[0]),
                "total_ay": float(total_cmd[1]),
                "total_norm": float(np.linalg.norm(total_cmd)),
            }
        )

        state.velocity = state.velocity + accel * cfg.dt
        state.velocity = _clip_norm(state.velocity, cfg.max_speed_norm)
        state.position = state.position + state.velocity * cfg.dt

        if float(np.linalg.norm(goal - state.position)) < 0.15 and float(np.linalg.norm(state.velocity)) < 0.1:
            break
        if observation.signed_clearance <= 0.0:
            break

    return history


def _plot_trajectory(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]]) -> None:
    colors = {
        "paper_pd": "#1f77b4",
        "paper_geometric": "#9467bd",
    }
    for name, history in histories.items():
        xs = [row["x"] for row in history]
        ys = [row["y"] for row in history]
        ax.plot(xs, ys, linewidth=2.0, label=name.upper(), color=colors[name])
        ax.scatter(xs[-1], ys[-1], color=colors[name], s=55, marker="x")

    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, PolygonObstacle):
            patch = PolygonPatch(obstacle.vertices, closed=True, color="#d62728", alpha=0.18)
            ax.add_patch(patch)

    ax.scatter(scenario.start[0], scenario.start[1], color="#2ca02c", s=70, marker="o", label="start")
    ax.scatter(scenario.goal[0], scenario.goal[1], color="#111111", s=90, marker="*", label="goal")
    ax.set_title("Nonconvex U-shape Trajectory")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)


def _plot_timeseries(axs: list[plt.Axes], histories: dict[str, list[dict[str, float]]]) -> None:
    colors = {
        "paper_pd": "#1f77b4",
        "paper_geometric": "#9467bd",
    }
    metrics = [
        ("goal_distance", "Goal Distance"),
        ("obstacle_distance", "Obstacle Distance"),
        ("goal_weight", "Goal Relaxation Weight"),
        ("goal_norm", "Goal Command Norm"),
        ("boundary_norm", "Boundary Field Norm"),
        ("collision_norm", "Collision Field Norm"),
        ("heading_error", "Heading Error [rad]"),
    ]
    for ax, (key, title) in zip(axs, metrics):
        for name, history in histories.items():
            ax.plot([row["step"] for row in history], [row[key] for row in history], color=colors[name], label=name.upper())
        ax.set_title(title)
        ax.set_xlabel("step")
        ax.grid(True, alpha=0.25)


def main() -> None:
    scenario = next(s for s in make_default_scenarios() if s.name == "nonconvex_u_shape")
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(parents=True, exist_ok=True)

    pd_config = make_paper_pd_config()
    geometric_config = make_paper_geometric_config()

    pd_nav = ReferenceNavigator(pd_config)
    geometric_nav = ReferenceNavigator(geometric_config)

    pd_history = simulate_with_diagnostics(_initial_state(scenario.start), scenario.goal, scenario.obstacles, pd_nav)
    geometric_history = simulate_with_diagnostics(
        _initial_state(scenario.start), scenario.goal, scenario.obstacles, geometric_nav
    )
    histories = {
        "paper_pd": pd_history,
        "paper_geometric": geometric_history,
    }

    for name, history in histories.items():
        metrics = compute_metrics(history, scenario.goal)
        print(
            f"{name}: success={int(metrics['success'])} "
            f"goal_reached_once={int(metrics['goal_reached_once'])} "
            f"final_goal_distance={metrics['final_goal_distance']:.3f} "
            f"min_clearance={metrics['min_clearance']:.3f} "
            f"path_length={metrics['path_length']:.3f}"
        )

    csv_path = artifacts / "nonconvex_diagnostic.csv"
    fieldnames = ["controller"] + list(pd_history[0].keys())
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for controller, history in histories.items():
            for row in history:
                writer.writerow({"controller": controller, **row})

    fig = plt.figure(figsize=(15, 12))
    grid = fig.add_gridspec(4, 2, height_ratios=[1.2, 1, 1, 1])
    trajectory_ax = fig.add_subplot(grid[0, :])
    series_axes = [fig.add_subplot(grid[i, j]) for i in range(1, 4) for j in range(2)]

    _plot_trajectory(trajectory_ax, scenario, histories)
    _plot_timeseries(series_axes, histories)

    handles, labels = trajectory_ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    plot_path = artifacts / "nonconvex_diagnostic.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    print(f"nonconvex_diagnostic_plot={plot_path}")
    print(f"nonconvex_diagnostic_csv={csv_path}")


if __name__ == "__main__":
    main()
