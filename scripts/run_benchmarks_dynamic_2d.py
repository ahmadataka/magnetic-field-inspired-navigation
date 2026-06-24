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
from matplotlib.patches import Circle as CirclePatch
from matplotlib.patches import Polygon as PolygonPatch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mfinav import (  # noqa: E402
    ArtificialPotentialFieldNavigator,
    DoubleIntegratorState,
    HaddadinNavigator,
    ReferenceNavigator,
    SabattiniNavigator,
    compute_metrics,
    make_dynamic_scenarios_2d,
    make_paper_geometric_config,
    make_paper_pd_config,
    simulate,
)


METHOD_SPECS = {
    "paper_pd": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric": {"label": "MFI-Geometric", "color": "#2ca02c"},
    "apf": {"label": "APF", "color": "#ff7f0e"},
    "haddadin": {"label": "Haddadin", "color": "#8b5cf6"},
    "sabattini": {"label": "Sabattini", "color": "#d97706"},
}


def _initial_state(start: np.ndarray) -> DoubleIntegratorState:
    return DoubleIntegratorState(position=start.copy(), velocity=np.array([0.0, 0.0], dtype=float))


def _make_metrics_row(scenario_name: str, method_name: str, metrics: dict[str, float]) -> dict[str, str | float]:
    return {
        "scenario": scenario_name,
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


def _sample_indices(max_len: int, target_frames: int = 60) -> list[int]:
    if max_len <= 1:
        return [0]
    count = min(target_frames, max_len)
    return sorted({int(round(value)) for value in np.linspace(0, max_len - 1, count)})


def _circle_points(center: list[float], radius: float, samples: int = 36) -> tuple[list[float], list[float]]:
    angles = np.linspace(0.0, 2.0 * math.pi, samples)
    xs = [center[0] + radius * math.cos(angle) for angle in angles]
    ys = [center[1] + radius * math.sin(angle) for angle in angles]
    return xs, ys


def _geometry_to_xy(shape: dict[str, object]) -> tuple[list[float], list[float]]:
    kind = shape["kind"]
    if kind == "circle":
        center = shape["center"]
        radius = float(shape["radius"])
        return _circle_points(center, radius)
    if kind == "polygon":
        vertices = shape["vertices"]
        xs = [float(vertex[0]) for vertex in vertices] + [float(vertices[0][0])]
        ys = [float(vertex[1]) for vertex in vertices] + [float(vertices[0][1])]
        return xs, ys
    raise ValueError(f"Unsupported shape kind {kind}.")


def _plot_static_snapshot(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]], sample_times: list[float]) -> None:
    for method_name, history in histories.items():
        color = METHOD_SPECS[method_name]["color"]
        xs = [row["x"] for row in history]
        ys = [row["y"] for row in history]
        ax.plot(xs, ys, linewidth=2.0, color=color, label=METHOD_SPECS[method_name]["label"])
        ax.scatter(xs[-1], ys[-1], color=color, s=48, marker="x")

    for idx, time_s in enumerate(sample_times):
        scenario.obstacles.set_time(time_s)
        snapshot = scenario.obstacles.snapshot()
        alpha = 0.08 + 0.08 * idx
        for shape in snapshot["obstacles"]:
            if shape["kind"] == "circle":
                ax.add_patch(
                    CirclePatch(
                        tuple(shape["center"]),
                        float(shape["radius"]),
                        color="#d62728",
                        alpha=alpha,
                    )
                )
            elif shape["kind"] == "polygon":
                ax.add_patch(
                    PolygonPatch(
                        np.asarray(shape["vertices"], dtype=float),
                        closed=True,
                        color="#d62728",
                        alpha=alpha,
                    )
                )

    ax.scatter(scenario.start[0], scenario.start[1], color="#0f9d58", s=75, marker="o", label="start")
    ax.scatter(scenario.goal[0], scenario.goal[1], color="#111111", s=95, marker="*", label="goal")
    ax.set_title(scenario.name.replace("_", " "))
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)


def _metrics_table_html(metrics_rows: list[dict[str, str | float]]) -> str:
    lines = [
        "<table class=\"metrics\">",
        "<thead><tr><th>Method</th><th>Success</th><th>Reach Once</th><th>Final Error</th><th>Min Clearance</th><th>Collision</th><th>Safe Viol</th><th>Path Length</th><th>T_goal (steps)</th></tr></thead>",
        "<tbody>",
    ]
    for row in metrics_rows:
        time_value = row["time_to_goal_steps"]
        time_text = "inf" if time_value == float("inf") else f"{time_value:.0f}"
        lines.append(
            "<tr>"
            f"<td>{row['method']}</td>"
            f"<td>{int(row['success'])}</td>"
            f"<td>{int(row['goal_reached_once'])}</td>"
            f"<td>{row['final_goal_distance']:.3f}</td>"
            f"<td>{row['min_clearance']:.3f}</td>"
            f"<td>{int(row['collision'])}</td>"
            f"<td>{int(row['safety_violation'])}</td>"
            f"<td>{row['path_length']:.3f}</td>"
            f"<td>{time_text}</td>"
            "</tr>"
        )
    lines.extend(["</tbody>", "</table>"])
    return "".join(lines)


def _build_interactive_html(scenarios_data: list[dict[str, object]]) -> str:
    html_parts = [
        "<!DOCTYPE html>",
        "<html lang=\"en\">",
        "<head>",
        "  <meta charset=\"utf-8\">",
        "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">",
        "  <title>Dynamic 2D Benchmark Comparison</title>",
        "  <script src=\"https://cdn.plot.ly/plotly-2.35.2.min.js\"></script>",
        "  <style>",
        "    body { font-family: Helvetica, Arial, sans-serif; margin: 0; background: #f7f7f5; color: #1a1a1a; }",
        "    main { max-width: 1500px; margin: 0 auto; padding: 24px; }",
        "    h1 { margin: 0 0 8px; font-size: 30px; }",
        "    h2 { margin: 0 0 8px; font-size: 22px; }",
        "    p { margin: 0 0 18px; line-height: 1.5; }",
        "    .scenario { background: #ffffff; border: 1px solid #dddddd; border-radius: 16px; padding: 18px; margin: 0 0 22px; box-shadow: 0 10px 24px rgba(0, 0, 0, 0.05); }",
        "    .plot { width: 100%; height: 760px; }",
        "    .caption { color: #555555; margin: 0 0 10px; }",
        "    .controls { display: flex; gap: 10px; margin: 0 0 14px; flex-wrap: wrap; align-items: center; }",
        "    .controls button { border: 1px solid #c9c9c9; background: #f3f4f6; color: #222222; border-radius: 999px; padding: 8px 14px; cursor: pointer; font-size: 14px; }",
        "    .controls button.active { background: #111827; color: #ffffff; border-color: #111827; }",
        "    .controls button.utility { background: #ffffff; }",
        "    .metrics { width: 100%; border-collapse: collapse; margin-top: 14px; font-size: 14px; }",
        "    .metrics th, .metrics td { border-bottom: 1px solid #e5e7eb; padding: 8px 10px; text-align: right; }",
        "    .metrics th:first-child, .metrics td:first-child { text-align: left; }",
        "    .metrics th { background: #f9fafb; }",
        "  </style>",
        "</head>",
        "<body>",
        "  <main>",
        "    <h1>Interactive Dynamic 2D Benchmark Comparison</h1>",
        "    <p>Each scene shows a moving obstacle field together with the robot trajectories. Use Play/Pause to animate the obstacle motion, and toggle algorithms to compare only the methods you want to inspect. Blue is MFI-PD, green is MFI-Geometric, orange is APF, purple is Haddadin, amber is Sabattini, teal is the start, and black is the goal.</p>",
    ]

    script_lines = [
        "<script>",
        "function flattenNumeric(values) {",
        "  if (!Array.isArray(values)) {",
        "    return typeof values === 'number' ? [values] : [];",
        "  }",
        "  const result = [];",
        "  const stack = [...values];",
        "  while (stack.length > 0) {",
        "    const value = stack.pop();",
        "    if (Array.isArray(value)) {",
        "      for (let i = 0; i < value.length; i += 1) stack.push(value[i]);",
        "    } else if (typeof value === 'number' && Number.isFinite(value)) {",
        "      result.push(value);",
        "    }",
        "  }",
        "  return result;",
        "}",
        "function computeRanges(traces, visibleMask) {",
        "  const xs = [];",
        "  const ys = [];",
        "  traces.forEach((trace, index) => {",
        "    if (!visibleMask[index]) return;",
        "    xs.push(...flattenNumeric(trace.x || []));",
        "    ys.push(...flattenNumeric(trace.y || []));",
        "  });",
        "  if (!xs.length || !ys.length) return null;",
        "  const makeRange = (values) => {",
        "    const min = Math.min(...values);",
        "    const max = Math.max(...values);",
        "    const span = Math.max(max - min, 1.0);",
        "    const pad = 0.14 * span;",
        "    return [min - pad, max + pad];",
        "  };",
        "  return {'xaxis.range': makeRange(xs), 'yaxis.range': makeRange(ys)};",
        "}",
        "function selectedAlgorithmsForPlot(plotId) {",
        "  return Array.from(document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"].active`)).map((button) => button.dataset.algorithm);",
        "}",
        "function applyAlgorithmSelection(plotId, traces) {",
        "  const selectedAlgorithms = selectedAlgorithmsForPlot(plotId);",
        "  const visibleMask = traces.map((trace) => {",
        "    const meta = trace.meta || {};",
        "    if (meta.category !== 'algorithm') return true;",
        "    return selectedAlgorithms.includes(meta.algorithm);",
        "  });",
        "  Plotly.restyle(plotId, {visible: visibleMask});",
        "  const ranges = computeRanges(traces, visibleMask);",
        "  if (ranges) Plotly.relayout(plotId, ranges);",
        "  document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((button) => {",
        "    button.classList.toggle('active', selectedAlgorithms.includes(button.dataset.algorithm));",
        "  });",
        "}",
        "function refreshPlotSelection(plotId) {",
        "  const index = Number(plotId.split('-')[1]);",
        "  applyAlgorithmSelection(plotId, window[`data${index}`]);",
        "}",
    ]

    for index, scenario_data in enumerate(scenarios_data):
        div_id = f"plot-{index}"
        html_parts.append("    <section class=\"scenario\">")
        html_parts.append(f"      <h2>{scenario_data['name']}</h2>")
        html_parts.append(f"      <p class=\"caption\">{scenario_data['description']}</p>")
        html_parts.append("      <div class=\"controls\">")
        html_parts.append(f"        <button class=\"utility\" data-plot=\"{div_id}\" data-action=\"play\">Play</button>")
        html_parts.append(f"        <button class=\"utility\" data-plot=\"{div_id}\" data-action=\"pause\">Pause</button>")
        html_parts.append(f"        <button class=\"utility\" data-plot=\"{div_id}\" data-action=\"all\">All</button>")
        html_parts.append(f"        <button class=\"utility\" data-plot=\"{div_id}\" data-action=\"none\">None</button>")
        for method_name, spec in METHOD_SPECS.items():
            html_parts.append(f"        <button class=\"active\" data-role=\"toggle\" data-plot=\"{div_id}\" data-algorithm=\"{method_name}\">{spec['label']}</button>")
        html_parts.append("      </div>")
        html_parts.append(f"      <div id=\"{div_id}\" class=\"plot\"></div>")
        html_parts.append(scenario_data["metrics_table"])
        html_parts.append("    </section>")

        script_lines.append(f"const data{index} = {json.dumps(scenario_data['plot_traces'], separators=(',', ':'))};")
        script_lines.append(f"window.data{index} = data{index};")
        script_lines.append(f"const layout{index} = {json.dumps(scenario_data['layout'], separators=(',', ':'))};")
        script_lines.append(f"const frames{index} = {json.dumps(scenario_data['frames'], separators=(',', ':'))};")
        script_lines.append(f"Plotly.newPlot('{div_id}', data{index}, layout{index}, {{responsive: true, displaylogo: false}}).then(() => {{")
        script_lines.append(f"  Plotly.addFrames('{div_id}', frames{index});")
        script_lines.append(f"  refreshPlotSelection('{div_id}');")
        script_lines.append("});")

    script_lines.append("document.querySelectorAll('.controls button').forEach((button) => {")
    script_lines.append("  button.addEventListener('click', () => {")
    script_lines.append("    const plotId = button.dataset.plot;")
    script_lines.append("    if (button.dataset.action === 'play') {")
    script_lines.append("      Plotly.animate(plotId, null, {fromcurrent: true, frame: {duration: 80, redraw: true}, transition: {duration: 0}});")
    script_lines.append("      return;")
    script_lines.append("    }")
    script_lines.append("    if (button.dataset.action === 'pause') {")
    script_lines.append("      Plotly.animate(plotId, [[null]], {mode: 'immediate', frame: {duration: 0, redraw: false}, transition: {duration: 0}});")
    script_lines.append("      return;")
    script_lines.append("    }")
    script_lines.append("    if (button.dataset.action === 'all') {")
    script_lines.append("      document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((toggle) => toggle.classList.add('active'));")
    script_lines.append("      refreshPlotSelection(plotId);")
    script_lines.append("      return;")
    script_lines.append("    }")
    script_lines.append("    if (button.dataset.action === 'none') {")
    script_lines.append("      document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((toggle) => toggle.classList.remove('active'));")
    script_lines.append("      refreshPlotSelection(plotId);")
    script_lines.append("      return;")
    script_lines.append("    }")
    script_lines.append("    if (button.dataset.role === 'toggle') {")
    script_lines.append("      const activeCount = document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"].active`).length;")
    script_lines.append("      const willDeactivate = button.classList.contains('active');")
    script_lines.append("      if (willDeactivate && activeCount === 1) return;")
    script_lines.append("      button.classList.toggle('active');")
    script_lines.append("      refreshPlotSelection(plotId);")
    script_lines.append("    }")
    script_lines.append("  });")
    script_lines.append("});")
    script_lines.append("</script>")

    html_parts.extend(script_lines)
    html_parts.extend(["  </main>", "</body>", "</html>"])
    return "\n".join(html_parts)


def main() -> None:
    scenarios = make_dynamic_scenarios_2d()
    config_pd = make_paper_pd_config()
    config_geometric = make_paper_geometric_config()
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
            "paper_pd": simulate(
                _initial_state(scenario.start),
                scenario.goal,
                scenario.obstacles,
                config_pd,
                navigator=ReferenceNavigator(config_pd),
            ),
            "paper_geometric": simulate(
                _initial_state(scenario.start),
                scenario.goal,
                scenario.obstacles,
                config_geometric,
                navigator=ReferenceNavigator(config_geometric),
            ),
            "apf": simulate(
                _initial_state(scenario.start),
                scenario.goal,
                scenario.obstacles,
                config_pd,
                navigator=ArtificialPotentialFieldNavigator(config_pd),
            ),
            "haddadin": simulate(
                _initial_state(scenario.start),
                scenario.goal,
                scenario.obstacles,
                config_pd,
                navigator=HaddadinNavigator(config_pd),
            ),
            "sabattini": simulate(
                _initial_state(scenario.start),
                scenario.goal,
                scenario.obstacles,
                config_pd,
                navigator=SabattiniNavigator(config_pd),
            ),
        }

        max_len = max(len(history) for history in histories.values())
        frame_indices = _sample_indices(max_len, target_frames=55)
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

            for method_name, history in histories.items():
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
                    "layout": {
                        "title": {
                            "text": f"{scenario.name.replace('_', ' ')} - t = {time_samples[frame_id]:.2f} s"
                        }
                    },
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
                                    "args": [
                                        [f"frame-{frame_id}"],
                                        {"mode": "immediate", "frame": {"duration": 0, "redraw": True}, "transition": {"duration": 0}},
                                    ],
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
    plot_path = artifacts / "benchmark_comparison_dynamic_2d.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    csv_path = artifacts / "benchmark_metrics_dynamic_2d.csv"
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

    html_path = artifacts / "benchmark_comparison_dynamic_2d.html"
    html_path.write_text(_build_interactive_html(interactive_scenarios), encoding="utf-8")

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
    print(f"benchmark_html={html_path}")


if __name__ == "__main__":
    main()
