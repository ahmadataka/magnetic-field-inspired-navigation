#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import csv
import json
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mfinav import (  # noqa: E402
    ArtificialPotentialFieldNavigator,
    HaddadinNavigator,
    MagneticFieldNavigator3D,
    QuadrotorState,
    SabattiniNavigator,
    compute_metrics,
    make_dynamic_scenarios_3d,
    make_paper_geometric_3d_config,
    make_paper_pd_3d_config,
    simulate_quadrotor,
)
from run_benchmarks_dynamic_3d import (  # noqa: E402
    METHOD_SPECS,
    _build_interactive_html,
    _downsample_history,
    _make_range,
    _metrics_table_html,
    _plot_projection,
    _sample_indices,
    _shape_to_wireframe,
)


def _initial_state(start: np.ndarray, gravity: float) -> QuadrotorState:
    return QuadrotorState(
        position=start.copy(),
        velocity=np.zeros(3, dtype=float),
        rotation=np.eye(3, dtype=float),
        angular_velocity=np.zeros(3, dtype=float),
        thrust_total=gravity,
        thrust_rate=0.0,
    )


def main() -> None:
    scenarios = make_dynamic_scenarios_3d()
    config_pd = make_paper_pd_3d_config()
    config_geometric = make_paper_geometric_3d_config()
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(parents=True, exist_ok=True)
    summary_rows: list[dict[str, str | float]] = []
    interactive_scenarios: list[dict[str, object]] = []
    fig, axes = plt.subplots(len(scenarios), 3, figsize=(16, 4.8 * len(scenarios)))
    axes = np.atleast_2d(axes)

    for row_axes, scenario in zip(axes, scenarios):
        histories = {
            "paper_pd_3d": simulate_quadrotor(_initial_state(scenario.start, config_pd.quadrotor_gravity), scenario.goal, scenario.obstacles, config_pd, navigator=MagneticFieldNavigator3D(config_pd)),
            "paper_geometric_3d": simulate_quadrotor(_initial_state(scenario.start, config_geometric.quadrotor_gravity), scenario.goal, scenario.obstacles, config_geometric, navigator=MagneticFieldNavigator3D(config_geometric)),
            "apf_3d": simulate_quadrotor(_initial_state(scenario.start, config_pd.quadrotor_gravity), scenario.goal, scenario.obstacles, config_pd, navigator=ArtificialPotentialFieldNavigator(config_pd)),
            "haddadin_3d": simulate_quadrotor(_initial_state(scenario.start, config_pd.quadrotor_gravity), scenario.goal, scenario.obstacles, config_pd, navigator=HaddadinNavigator(config_pd)),
            "sabattini_3d": simulate_quadrotor(_initial_state(scenario.start, config_pd.quadrotor_gravity), scenario.goal, scenario.obstacles, config_pd, navigator=SabattiniNavigator(config_pd)),
        }
        max_len = max(len(history) for history in histories.values())
        frame_indices = _sample_indices(max_len, target_frames=28)
        time_samples = [index * config_pd.dt for index in frame_indices]
        obstacle_snapshots = scenario.obstacles.snapshots_over_time(time_samples)
        static_snapshot = obstacle_snapshots[min(len(obstacle_snapshots) - 1, len(obstacle_snapshots) // 2)]

        _plot_projection(row_axes[0], scenario, histories, ("x", "y"), static_snapshot)
        row_axes[0].set_title(f"{scenario.name} (xy)")
        _plot_projection(row_axes[1], scenario, histories, ("x", "z"), static_snapshot)
        row_axes[1].set_title("xz")
        _plot_projection(row_axes[2], scenario, histories, ("y", "z"), static_snapshot)
        row_axes[2].set_title("yz")

        metrics_rows_for_html: list[dict[str, str | float]] = []
        plot_traces: list[dict[str, object]] = []
        obstacle_trace_count = len(obstacle_snapshots[0]["obstacles"]) if obstacle_snapshots else 0
        for obstacle_index in range(obstacle_trace_count):
            xs, ys, zs = _shape_to_wireframe(obstacle_snapshots[0]["obstacles"][obstacle_index])
            plot_traces.append({"type": "scatter3d", "mode": "lines", "x": xs, "y": ys, "z": zs, "line": {"color": "#ef4444", "width": 5}, "name": f"Obstacle {obstacle_index + 1}", "hoverinfo": "skip", "meta": {"category": "context"}, "showlegend": obstacle_index == 0})
        downsampled_histories = {name: _downsample_history(history, max_points=360) for name, history in histories.items()}
        for method_name, history in downsampled_histories.items():
            spec = METHOD_SPECS[method_name]
            x0 = [history[0]["x"]]
            y0 = [history[0]["y"]]
            z0 = [history[0]["z"]]
            plot_traces.append({"type": "scatter3d", "mode": "lines", "x": x0, "y": y0, "z": z0, "line": {"color": spec["color"], "width": 6}, "name": spec["label"], "meta": {"category": "algorithm", "algorithm": method_name}})
            plot_traces.append({"type": "scatter3d", "mode": "markers", "x": x0, "y": y0, "z": z0, "marker": {"color": spec["color"], "size": 4, "symbol": "x"}, "name": f"{spec['label']} current", "showlegend": False, "hoverinfo": "skip", "meta": {"category": "algorithm", "algorithm": method_name}})
            metrics = compute_metrics(history, scenario.goal, success_radius=config_pd.quadrotor_goal_tolerance)
            row = {"scenario": scenario.name, "method": method_name, "success": metrics["success"], "goal_reached_once": metrics["goal_reached_once"], "steps": metrics["steps"], "path_length": metrics["path_length"], "final_goal_distance": metrics["final_goal_distance"], "min_clearance": metrics["min_clearance"], "mean_speed": metrics["mean_speed"], "collision": metrics["collision"], "safety_violation": metrics["safety_violation"], "time_to_goal_steps": metrics["time_to_goal_steps"], "path_efficiency": metrics["path_efficiency"]}
            summary_rows.append(row)
            metrics_rows_for_html.append(row)
        plot_traces.extend([
            {"type": "scatter3d", "mode": "markers", "x": [float(scenario.start[0])], "y": [float(scenario.start[1])], "z": [float(scenario.start[2])], "marker": {"color": "#0f766e", "size": 5}, "name": "Start", "meta": {"category": "static"}},
            {"type": "scatter3d", "mode": "markers", "x": [float(scenario.goal[0])], "y": [float(scenario.goal[1])], "z": [float(scenario.goal[2])], "marker": {"color": "#111111", "size": 6, "symbol": "diamond"}, "name": "Goal", "meta": {"category": "static"}},
        ])
        frames: list[dict[str, object]] = []
        for frame_id, history_index in enumerate(frame_indices):
            frame_traces: list[dict[str, object]] = []
            for obstacle_shape in obstacle_snapshots[frame_id]["obstacles"]:
                xs, ys, zs = _shape_to_wireframe(obstacle_shape)
                frame_traces.append({"x": xs, "y": ys, "z": zs})
            for history in downsampled_histories.values():
                clamped = min(history_index, len(history) - 1)
                xs = [row["x"] for row in history[: clamped + 1]]
                ys = [row["y"] for row in history[: clamped + 1]]
                zs = [row["z"] for row in history[: clamped + 1]]
                frame_traces.append({"x": xs, "y": ys, "z": zs})
                frame_traces.append({"x": [xs[-1]], "y": [ys[-1]], "z": [zs[-1]]})
            frames.append({"name": f"frame-{frame_id}", "data": frame_traces, "traces": list(range(obstacle_trace_count + 2 * len(downsampled_histories))), "layout": {"title": {"text": f"{scenario.name.replace('_', ' ')} - t = {time_samples[frame_id]:.2f} s"}}})
        all_x = [float(scenario.start[0]), float(scenario.goal[0])]
        all_y = [float(scenario.start[1]), float(scenario.goal[1])]
        all_z = [float(scenario.start[2]), float(scenario.goal[2])]
        for history in downsampled_histories.values():
            all_x.extend(row["x"] for row in history)
            all_y.extend(row["y"] for row in history)
            all_z.extend(row["z"] for row in history)
        for snapshot in obstacle_snapshots:
            for shape in snapshot["obstacles"]:
                xs, ys, zs = _shape_to_wireframe(shape)
                all_x.extend(value for value in xs if value is not None)
                all_y.extend(value for value in ys if value is not None)
                all_z.extend(value for value in zs if value is not None)
        interactive_scenarios.append({"name": scenario.name.replace("_", " "), "description": scenario.description, "plot_traces": plot_traces, "frames": frames, "metrics_table": _metrics_table_html(metrics_rows_for_html), "layout": {"title": {"text": f"{scenario.name.replace('_', ' ')} - t = 0.00 s"}, "scene": {"xaxis": {"title": "x", "range": _make_range(all_x)}, "yaxis": {"title": "y", "range": _make_range(all_y)}, "zaxis": {"title": "z", "range": _make_range(all_z)}, "aspectmode": "data"}, "legend": {"orientation": "h", "yanchor": "bottom", "y": 1.02, "x": 0.0}, "margin": {"l": 0, "r": 0, "t": 70, "b": 0}, "sliders": [{"active": 0, "currentvalue": {"prefix": "Frame: "}, "pad": {"t": 18}, "steps": [{"label": str(frame_id), "method": "animate", "args": [[f"frame-{frame_id}"], {"mode": "immediate", "frame": {"duration": 0, "redraw": True}, "transition": {"duration": 0}}]} for frame_id in range(len(frames))]}]}})

    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=5, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    plot_path = artifacts / "benchmark_comparison_dynamic_quadrotor_3d.png"
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    csv_path = artifacts / "benchmark_metrics_dynamic_quadrotor_3d.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["scenario", "method", "success", "goal_reached_once", "steps", "path_length", "final_goal_distance", "min_clearance", "mean_speed", "collision", "safety_violation", "time_to_goal_steps", "path_efficiency"])
        writer.writeheader()
        writer.writerows(summary_rows)
    html_path = artifacts / "benchmark_comparison_dynamic_quadrotor_3d.html"
    html_path.write_text(_build_interactive_html(interactive_scenarios), encoding="utf-8")
    print(f"benchmark_plot_dynamic_quadrotor_3d={plot_path}")
    print(f"benchmark_metrics_dynamic_quadrotor_3d={csv_path}")
    print(f"benchmark_html_dynamic_quadrotor_3d={html_path}")


if __name__ == "__main__":
    main()
