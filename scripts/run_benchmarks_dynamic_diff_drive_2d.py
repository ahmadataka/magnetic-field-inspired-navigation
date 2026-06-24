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
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mfinav import (  # noqa: E402
    ArtificialPotentialFieldNavigator,
    DifferentialDriveState,
    HaddadinNavigator,
    ReferenceNavigator,
    SabattiniNavigator,
    compute_metrics,
    make_dynamic_scenarios_2d,
    make_paper_geometric_config,
    make_paper_pd_config,
    simulate_differential_drive,
)
from run_benchmarks_dynamic_2d import (  # noqa: E402
    METHOD_SPECS,
    _build_interactive_html,
    _geometry_to_xy,
    _make_metrics_row,
    _metrics_table_html,
    _plot_static_snapshot,
    _sample_indices,
)


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


def main() -> None:
    scenarios = make_dynamic_scenarios_2d()
    config_pd = _diff_drive_config(make_paper_pd_config())
    config_geometric = _diff_drive_config(make_paper_geometric_config())
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(parents=True, exist_ok=True)

    summary_rows: list[dict[str, str | float]] = []
    interactive_scenarios: list[dict[str, object]] = []

    cols = min(2, len(scenarios))
    rows = math.ceil(len(scenarios) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(7 * cols, 5.4 * rows))
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

        max_len = max(len(history) for history in histories.values())
        frame_indices = _sample_indices(max_len, target_frames=30)
        time_samples = [index * config_pd.dt for index in frame_indices]
        obstacle_snapshots = scenario.obstacles.snapshots_over_time(time_samples)

        sample_times_for_png = np.linspace(0.0, max(time_samples), 4).tolist() if time_samples else [0.0]
        _plot_static_snapshot(ax, scenario, histories, sample_times_for_png)

        metrics_rows_for_html: list[dict[str, str | float]] = []
        plot_traces: list[dict[str, object]] = []
        obstacle_trace_count = len(obstacle_snapshots[0]["obstacles"]) if obstacle_snapshots else 0

        for obstacle_index in range(obstacle_trace_count):
            xs, ys = _geometry_to_xy(obstacle_snapshots[0]["obstacles"][obstacle_index])
            plot_traces.append(
                {
                    "type": "scatter",
                    "mode": "lines",
                    "x": xs,
                    "y": ys,
                    "fill": "toself",
                    "line": {"color": "rgba(214,39,40,0.85)", "width": 2},
                    "fillcolor": "rgba(214,39,40,0.16)",
                    "name": f"Obstacle {obstacle_index + 1}",
                    "hoverinfo": "skip",
                    "meta": {"category": "obstacle"},
                    "showlegend": obstacle_index == 0,
                }
            )

        for method_name, history in histories.items():
            spec = METHOD_SPECS[method_name]
            x0 = [history[0]["x"]]
            y0 = [history[0]["y"]]
            plot_traces.append(
                {
                    "type": "scatter",
                    "mode": "lines",
                    "x": x0,
                    "y": y0,
                    "line": {"color": spec["color"], "width": 3},
                    "name": spec["label"],
                    "meta": {"category": "algorithm", "algorithm": method_name},
                }
            )
            plot_traces.append(
                {
                    "type": "scatter",
                    "mode": "markers",
                    "x": x0,
                    "y": y0,
                    "marker": {"color": spec["color"], "size": 9, "symbol": "x"},
                    "name": f"{spec['label']} current",
                    "showlegend": False,
                    "hoverinfo": "skip",
                    "meta": {"category": "algorithm", "algorithm": method_name},
                }
            )
            metrics = compute_metrics(history, scenario.goal)
            row = _make_metrics_row(scenario.name, method_name, metrics)
            summary_rows.append(row)
            metrics_rows_for_html.append(row)

        plot_traces.append(
            {
                "type": "scatter",
                "mode": "markers",
                "x": [float(scenario.start[0])],
                "y": [float(scenario.start[1])],
                "marker": {"color": "#0f9d58", "size": 11},
                "name": "Start",
                "meta": {"category": "static"},
            }
        )
        plot_traces.append(
            {
                "type": "scatter",
                "mode": "markers",
                "x": [float(scenario.goal[0])],
                "y": [float(scenario.goal[1])],
                "marker": {"color": "#111111", "size": 13, "symbol": "star"},
                "name": "Goal",
                "meta": {"category": "static"},
            }
        )

        frames: list[dict[str, object]] = []
        for frame_id, history_index in enumerate(frame_indices):
            frame_traces: list[dict[str, object]] = []
            for obstacle_shape in obstacle_snapshots[frame_id]["obstacles"]:
                xs, ys = _geometry_to_xy(obstacle_shape)
                frame_traces.append({"x": xs, "y": ys})
            for history in histories.values():
                clamped = min(history_index, len(history) - 1)
                xs = [row["x"] for row in history[: clamped + 1]]
                ys = [row["y"] for row in history[: clamped + 1]]
                frame_traces.append({"x": xs, "y": ys})
                frame_traces.append({"x": [xs[-1]], "y": [ys[-1]]})
            frames.append(
                {
                    "name": f"frame-{frame_id}",
                    "data": frame_traces,
                    "traces": list(range(obstacle_trace_count + 2 * len(histories))),
                    "layout": {"title": {"text": f"{scenario.name.replace('_', ' ')} - t = {time_samples[frame_id]:.2f} s"}},
                }
            )

        all_x = [float(scenario.start[0]), float(scenario.goal[0])]
        all_y = [float(scenario.start[1]), float(scenario.goal[1])]
        for history in histories.values():
            all_x.extend(row["x"] for row in history)
            all_y.extend(row["y"] for row in history)
        for snapshot in obstacle_snapshots:
            for shape in snapshot["obstacles"]:
                xs, ys = _geometry_to_xy(shape)
                all_x.extend(xs)
                all_y.extend(ys)
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        x_span = max(max_x - min_x, 1.0)
        y_span = max(max_y - min_y, 1.0)
        x_pad = 0.15 * x_span
        y_pad = 0.15 * y_span

        interactive_scenarios.append(
            {
                "name": scenario.name.replace("_", " "),
                "description": scenario.description,
                "plot_traces": plot_traces,
                "frames": frames,
                "metrics_table": _metrics_table_html(metrics_rows_for_html),
                "layout": {
                    "title": {"text": f"{scenario.name.replace('_', ' ')} - t = 0.00 s"},
                    "paper_bgcolor": "#ffffff",
                    "plot_bgcolor": "#ffffff",
                    "hovermode": "closest",
                    "xaxis": {"title": "x", "range": [min_x - x_pad, max_x + x_pad], "scaleanchor": "y", "scaleratio": 1.0},
                    "yaxis": {"title": "y", "range": [min_y - y_pad, max_y + y_pad]},
                    "legend": {"orientation": "h", "yanchor": "bottom", "y": 1.02, "x": 0.0},
                    "margin": {"l": 50, "r": 20, "t": 70, "b": 50},
                    "sliders": [
                        {
                            "active": 0,
                            "currentvalue": {"prefix": "Frame: "},
                            "pad": {"t": 18},
                            "steps": [
                                {
                                    "label": str(frame_id),
                                    "method": "animate",
                                    "args": [[f"frame-{frame_id}"], {"mode": "immediate", "frame": {"duration": 0, "redraw": True}, "transition": {"duration": 0}}],
                                }
                                for frame_id in range(len(frames))
                            ],
                        }
                    ],
                },
            }
        )

    for ax in axes[len(scenarios):]:
        ax.axis("off")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    plot_path = artifacts / "benchmark_comparison_dynamic_diff_drive_2d.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    csv_path = artifacts / "benchmark_metrics_dynamic_diff_drive_2d.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["scenario", "method", "success", "goal_reached_once", "steps", "path_length", "final_goal_distance", "min_clearance", "mean_speed", "collision", "safety_violation", "time_to_goal_steps", "path_efficiency"])
        writer.writeheader()
        writer.writerows(summary_rows)

    html_path = artifacts / "benchmark_comparison_dynamic_diff_drive_2d.html"
    html_path.write_text(_build_interactive_html(interactive_scenarios), encoding="utf-8")

    print(f"benchmark_plot_dynamic_diff_drive_2d={plot_path}")
    print(f"benchmark_metrics_dynamic_diff_drive_2d={csv_path}")
    print(f"benchmark_html_dynamic_diff_drive_2d={html_path}")


if __name__ == "__main__":
    main()
