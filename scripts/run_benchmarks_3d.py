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

from mfinav import (
    ArtificialPotentialFieldNavigator,
    DoubleIntegratorState,
    MagneticFieldNavigator3D,
    PrismObstacle,
    SphereObstacle,
    compute_metrics,
    make_default_scenarios_3d,
    make_paper_geometric_3d_config,
    make_paper_pd_3d_config,
    simulate,
)

METHOD_SPECS = {
    "paper_pd_3d": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric_3d": {"label": "MFI-Geometric", "color": "#2ca02c"},
    "apf_3d": {"label": "APF", "color": "#ff7f0e"},
}


def _initial_state(start: np.ndarray) -> DoubleIntegratorState:
    return DoubleIntegratorState(position=start.copy(), velocity=np.zeros_like(start))


def _downsample_history(history: list[dict[str, float]], max_points: int = 1200) -> list[dict[str, float]]:
    if len(history) <= max_points:
        return history
    indices = np.linspace(0, len(history) - 1, max_points, dtype=int)
    return [history[index] for index in indices]


def _sphere_surface(center: np.ndarray, radius: float, n_theta: int = 18, n_phi: int = 10) -> tuple[list[list[float]], list[list[float]], list[list[float]]]:
    theta_values = np.linspace(0.0, 2.0 * math.pi, n_theta)
    phi_values = np.linspace(0.0, math.pi, n_phi)
    xs: list[list[float]] = []
    ys: list[list[float]] = []
    zs: list[list[float]] = []
    for phi in phi_values:
        row_x: list[float] = []
        row_y: list[float] = []
        row_z: list[float] = []
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
        for theta in theta_values:
            row_x.append(float(center[0] + radius * math.cos(theta) * sin_phi))
            row_y.append(float(center[1] + radius * math.sin(theta) * sin_phi))
            row_z.append(float(center[2] + radius * cos_phi))
        xs.append(row_x)
        ys.append(row_y)
        zs.append(row_z)
    return xs, ys, zs


def _prism_wireframe(prism: PrismObstacle) -> list[dict[str, object]]:
    vertices = prism.vertices_xy
    traces: list[dict[str, object]] = []
    bottom_loop = np.column_stack((vertices, np.full(len(vertices), prism.z_min)))
    top_loop = np.column_stack((vertices, np.full(len(vertices), prism.z_max)))
    for loop, name in ((bottom_loop, "bottom"), (top_loop, "top")):
        closed = np.vstack((loop, loop[0]))
        traces.append(
            {
                "type": "scatter3d",
                "mode": "lines",
                "name": f"prism_{name}",
                "x": closed[:, 0].tolist(),
                "y": closed[:, 1].tolist(),
                "z": closed[:, 2].tolist(),
                "line": {"color": "#ef4444", "width": 5},
                "hoverinfo": "skip",
                "showlegend": False,
                "meta": {"category": "context"},
            }
        )
    for vertex in vertices:
        traces.append(
            {
                "type": "scatter3d",
                "mode": "lines",
                "name": "prism_side",
                "x": [float(vertex[0]), float(vertex[0])],
                "y": [float(vertex[1]), float(vertex[1])],
                "z": [prism.z_min, prism.z_max],
                "line": {"color": "#ef4444", "width": 4},
                "hoverinfo": "skip",
                "showlegend": False,
                "meta": {"category": "context"},
            }
        )
    return traces


def _plot_prism_projection(ax: plt.Axes, prism: PrismObstacle, axes: tuple[str, str]) -> None:
    x_key, y_key = axes
    axis_map = {"x": 0, "y": 1, "z": 2}
    if axes == ("x", "y"):
        polygon = np.vstack((prism.vertices_xy, prism.vertices_xy[0]))
        ax.fill(polygon[:, 0], polygon[:, 1], color="#ef4444", alpha=0.18)
        ax.plot(polygon[:, 0], polygon[:, 1], color="#ef4444", linewidth=1.5, alpha=0.8)
        return

    min_x = float(np.min(prism.vertices_xy[:, axis_map[x_key]])) if x_key != "z" else prism.z_min
    max_x = float(np.max(prism.vertices_xy[:, axis_map[x_key]])) if x_key != "z" else prism.z_max
    min_y = float(np.min(prism.vertices_xy[:, axis_map[y_key]])) if y_key != "z" else prism.z_min
    max_y = float(np.max(prism.vertices_xy[:, axis_map[y_key]])) if y_key != "z" else prism.z_max

    rectangle_x = [min_x, max_x, max_x, min_x, min_x]
    rectangle_y = [min_y, min_y, max_y, max_y, min_y]
    ax.fill(rectangle_x, rectangle_y, color="#ef4444", alpha=0.12)
    ax.plot(rectangle_x, rectangle_y, color="#ef4444", linewidth=1.2, alpha=0.7)


def _plot_projection(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]], axes: tuple[str, str]) -> None:
    x_key, y_key = axes
    for name, history in histories.items():
        xs = [row[x_key] for row in history]
        ys = [row[y_key] for row in history]
        color = METHOD_SPECS[name]["color"]
        label = METHOD_SPECS[name]["label"]
        ax.plot(xs, ys, linewidth=2.0, label=label, color=color)
        ax.scatter(xs[-1], ys[-1], color=color, s=45, marker="x")

    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, SphereObstacle):
            center = obstacle.center
            ax.scatter(center[0 if x_key == "x" else 1 if x_key == "y" else 2], center[0 if y_key == "x" else 1 if y_key == "y" else 2], color="#d62728", s=80, alpha=0.35)
        elif isinstance(obstacle, PrismObstacle):
            _plot_prism_projection(ax, obstacle, axes)

    ax.scatter(
        scenario.start[0 if x_key == "x" else 1 if x_key == "y" else 2],
        scenario.start[0 if y_key == "x" else 1 if y_key == "y" else 2],
        color="#0f766e",
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


def _build_interactive_html(scenarios_data: list[dict[str, object]]) -> str:
    html_parts = [
        "<!DOCTYPE html>",
        "<html lang=\"en\">",
        "<head>",
        "  <meta charset=\"utf-8\">",
        "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">",
        "  <title>3D Benchmark Comparison</title>",
        "  <script src=\"https://cdn.plot.ly/plotly-2.35.2.min.js\"></script>",
        "  <style>",
        "    body { font-family: Helvetica, Arial, sans-serif; margin: 0; background: #f7f7f5; color: #1a1a1a; }",
        "    main { max-width: 1400px; margin: 0 auto; padding: 24px; }",
        "    h1 { margin: 0 0 8px; font-size: 28px; }",
        "    p { margin: 0 0 18px; line-height: 1.5; }",
        "    .scenario { background: #ffffff; border: 1px solid #dddddd; border-radius: 16px; padding: 18px; margin: 0 0 22px; box-shadow: 0 10px 24px rgba(0, 0, 0, 0.05); }",
        "    .plot { width: 100%; height: 760px; }",
        "    .caption { color: #555555; margin: 0 0 10px; }",
        "    .controls { display: flex; gap: 10px; margin: 0 0 14px; flex-wrap: wrap; align-items: center; }",
        "    .controls button { border: 1px solid #c9c9c9; background: #f3f4f6; color: #222222; border-radius: 999px; padding: 8px 14px; cursor: pointer; font-size: 14px; }",
        "    .controls button.active { background: #111827; color: #ffffff; border-color: #111827; }",
        "    .controls button.utility { background: #ffffff; }",
        "  </style>",
        "</head>",
        "<body>",
        "  <main>",
        "    <h1>Interactive 3D Benchmark Comparison</h1>",
        "    <p>Rotate, pan, and zoom each scene. Blue is MFI-PD, green is MFI-Geometric, orange is APF, teal is the start, black is the goal, and red obstacle geometry shows spheres or extruded polygon prisms.</p>",
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
        "function computeSceneRanges(traces, visibleMask) {",
        "  const xs = [];",
        "  const ys = [];",
        "  const zs = [];",
        "  traces.forEach((trace, index) => {",
        "    if (!visibleMask[index]) return;",
        "    xs.push(...flattenNumeric(trace.x || []));",
        "    ys.push(...flattenNumeric(trace.y || []));",
        "    zs.push(...flattenNumeric(trace.z || []));",
        "  });",
        "  if (!xs.length || !ys.length || !zs.length) return null;",
        "  const makeRange = (values) => {",
        "    const min = Math.min(...values);",
        "    const max = Math.max(...values);",
        "    const span = Math.max(max - min, 1.0);",
        "    const pad = 0.12 * span;",
        "    return [min - pad, max + pad];",
        "  };",
        "  return {",
        "    'scene.xaxis.range': makeRange(xs),",
        "    'scene.yaxis.range': makeRange(ys),",
        "    'scene.zaxis.range': makeRange(zs),",
        "    'scene.aspectmode': 'data'",
        "  };",
        "}",
        "function applyAlgorithmSelection(plotId, traces, selectedAlgorithms) {",
        "  const visibleMask = traces.map((trace) => {",
        "    const meta = trace.meta || {};",
        "    if (meta.category !== 'algorithm') return true;",
        "    return selectedAlgorithms.includes(meta.algorithm);",
        "  });",
        "  Plotly.restyle(plotId, {visible: visibleMask});",
        "  const ranges = computeSceneRanges(traces, visibleMask);",
        "  if (ranges) Plotly.relayout(plotId, ranges);",
        "  document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((button) => {",
        "    button.classList.toggle('active', selectedAlgorithms.includes(button.dataset.algorithm));",
        "  });",
        "}",
        "function selectedAlgorithmsForPlot(plotId) {",
        "  return Array.from(document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"].active`)).map((button) => button.dataset.algorithm);",
        "}",
        "function refreshPlotSelection(plotId) {",
        "  const index = Number(plotId.split('-')[1]);",
        "  const traces = window[`data${index}`];",
        "  const selected = selectedAlgorithmsForPlot(plotId);",
        "  applyAlgorithmSelection(plotId, traces, selected);",
        "}",
    ]
    for index, scenario_data in enumerate(scenarios_data):
        div_id = f"plot-{index}"
        html_parts.append(f"    <section class=\"scenario\">")
        html_parts.append(f"      <h2>{scenario_data['name']}</h2>")
        html_parts.append(f"      <p class=\"caption\">{scenario_data['description']}</p>")
        html_parts.append("      <div class=\"controls\">")
        html_parts.append(f"        <button class=\"utility\" data-plot=\"{div_id}\" data-action=\"all\">All</button>")
        html_parts.append(f"        <button class=\"utility\" data-plot=\"{div_id}\" data-action=\"none\">None</button>")
        for method_name, spec in METHOD_SPECS.items():
            html_parts.append(
                f"        <button class=\"active\" data-role=\"toggle\" data-plot=\"{div_id}\" data-algorithm=\"{method_name}\">{spec['label']}</button>"
            )
        html_parts.append("      </div>")
        html_parts.append(f"      <div id=\"{div_id}\" class=\"plot\"></div>")
        html_parts.append("    </section>")
        script_lines.append(
            f"const data{index} = {json.dumps(scenario_data['plot_traces'], separators=(',', ':'))};"
        )
        script_lines.append(
            f"window.data{index} = data{index};"
        )
        script_lines.append(
            f"const layout{index} = {json.dumps(scenario_data['layout'], separators=(',', ':'))};"
        )
        script_lines.append(
            f"Plotly.newPlot('{div_id}', data{index}, layout{index}, {{responsive: true, displaylogo: false}});"
        )
        script_lines.append(
            f"refreshPlotSelection('{div_id}');"
        )
    script_lines.append(
        "document.querySelectorAll('.controls button').forEach((button) => {"
    )
    script_lines.append(
        "  button.addEventListener('click', () => {"
    )
    script_lines.append(
        "    const plotId = button.dataset.plot;"
    )
    script_lines.append(
        "    if (button.dataset.action === 'all') {"
    )
    script_lines.append(
        "      document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((toggle) => toggle.classList.add('active'));"
    )
    script_lines.append(
        "      refreshPlotSelection(plotId);"
    )
    script_lines.append(
        "      return;"
    )
    script_lines.append(
        "    }"
    )
    script_lines.append(
        "    if (button.dataset.action === 'none') {"
    )
    script_lines.append(
        "      document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((toggle) => toggle.classList.remove('active'));"
    )
    script_lines.append(
        "      refreshPlotSelection(plotId);"
    )
    script_lines.append(
        "      return;"
    )
    script_lines.append(
        "    }"
    )
    script_lines.append(
        "    if (button.dataset.role === 'toggle') {"
    )
    script_lines.append(
        "      const activeToggles = document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"].active`).length;"
    )
    script_lines.append(
        "      const willDeactivate = button.classList.contains('active');"
    )
    script_lines.append(
        "      if (willDeactivate && activeToggles === 1) return;"
    )
    script_lines.append(
        "      button.classList.toggle('active');"
    )
    script_lines.append(
        "      refreshPlotSelection(plotId);"
    )
    script_lines.append(
        "    }"
    )
    script_lines.append("  });")
    script_lines.append("});")
    script_lines.append("</script>")

    html_parts.extend(script_lines)
    html_parts.extend(["  </main>", "</body>", "</html>"])
    return "\n".join(html_parts)


def main() -> None:
    scenarios = make_default_scenarios_3d()
    config_pd = make_paper_pd_3d_config()
    config_geometric = make_paper_geometric_3d_config()
    config_apf = make_paper_pd_3d_config()
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(parents=True, exist_ok=True)

    summary_rows: list[dict[str, str | float]] = []
    interactive_scenarios: list[dict[str, object]] = []
    fig, axes = plt.subplots(len(scenarios), 3, figsize=(16, 4.8 * len(scenarios)))
    axes = np.atleast_2d(axes)

    for row_axes, scenario in zip(axes, scenarios):
        history = simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            config_pd,
            navigator=MagneticFieldNavigator3D(config_pd),
        )
        geometric_history = simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            config_geometric,
            navigator=MagneticFieldNavigator3D(config_geometric),
        )
        apf_history = simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            config_apf,
            navigator=ArtificialPotentialFieldNavigator(config_apf),
        )
        histories = {
            "paper_pd_3d": history,
            "paper_geometric_3d": geometric_history,
            "apf_3d": apf_history,
        }

        plot_traces: list[dict[str, object]] = []
        for obstacle_index, obstacle in enumerate(scenario.obstacles.obstacles):
            if isinstance(obstacle, SphereObstacle):
                xs, ys, zs = _sphere_surface(obstacle.center, obstacle.radius)
                plot_traces.append(
                    {
                        "type": "surface",
                        "x": xs,
                        "y": ys,
                        "z": zs,
                        "opacity": 0.28,
                        "showscale": False,
                        "hoverinfo": "skip",
                        "name": f"obstacle_{obstacle_index + 1}",
                        "colorscale": [[0.0, "#ef4444"], [1.0, "#ef4444"]],
                        "meta": {"category": "context"},
                    }
                )
            elif isinstance(obstacle, PrismObstacle):
                plot_traces.extend(_prism_wireframe(obstacle))

        for ax, proj in zip(row_axes, (("x", "y"), ("x", "z"), ("y", "z"))):
            _plot_projection(ax, scenario, histories, proj)
            ax.set_title(f"{scenario.name} {proj[0]}{proj[1]}")

        for method_name, method_history in histories.items():
            reduced_history = _downsample_history(method_history)
            plot_traces.append(
                {
                    "type": "scatter3d",
                    "mode": "lines",
                    "name": METHOD_SPECS[method_name]["label"],
                    "x": [row["x"] for row in reduced_history],
                    "y": [row["y"] for row in reduced_history],
                    "z": [row["z"] for row in reduced_history],
                    "line": {
                        "color": METHOD_SPECS[method_name]["color"],
                        "width": 7 if method_name == "paper_pd_3d" else 6 if method_name == "paper_geometric_3d" else 5,
                    },
                    "meta": {"category": "algorithm", "algorithm": method_name},
                }
            )
            plot_traces.append(
                {
                    "type": "scatter3d",
                    "mode": "markers",
                    "name": f"{METHOD_SPECS[method_name]['label']} final",
                    "x": [reduced_history[-1]["x"]],
                    "y": [reduced_history[-1]["y"]],
                    "z": [reduced_history[-1]["z"]],
                    "marker": {
                        "size": 4,
                        "symbol": "x",
                        "color": METHOD_SPECS[method_name]["color"],
                    },
                    "meta": {"category": "algorithm", "algorithm": method_name},
                }
            )
            metrics = compute_metrics(method_history, scenario.goal)
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
        plot_traces.extend(
            [
                {
                    "type": "scatter3d",
                    "mode": "markers+text",
                    "name": "start",
                    "x": [float(scenario.start[0])],
                    "y": [float(scenario.start[1])],
                    "z": [float(scenario.start[2])],
                    "text": ["start"],
                    "textposition": "top center",
                    "marker": {"size": 7, "color": "#0f766e"},
                    "meta": {"category": "context"},
                },
                {
                    "type": "scatter3d",
                    "mode": "markers+text",
                    "name": "goal",
                    "x": [float(scenario.goal[0])],
                    "y": [float(scenario.goal[1])],
                    "z": [float(scenario.goal[2])],
                    "text": ["goal"],
                    "textposition": "top center",
                    "marker": {"size": 8, "color": "#111111", "symbol": "diamond"},
                    "meta": {"category": "context"},
                },
            ]
        )
        interactive_scenarios.append(
            {
                "name": scenario.name,
                "description": scenario.description,
                "plot_traces": plot_traces,
                "layout": {
                    "margin": {"l": 0, "r": 0, "t": 36, "b": 0},
                    "legend": {"orientation": "h", "y": 1.04},
                    "scene": {
                        "aspectmode": "data",
                        "xaxis": {"title": "x"},
                        "yaxis": {"title": "y"},
                        "zaxis": {"title": "z"},
                        "camera": {"eye": {"x": 1.6, "y": 1.4, "z": 0.9}},
                    },
                    "title": {"text": scenario.name.replace("_", " ")},
                },
            }
        )

    fig.tight_layout()
    plot_path = artifacts / "benchmark_comparison_3d.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    interactive_path = artifacts / "benchmark_comparison_3d.html"
    interactive_path.write_text(_build_interactive_html(interactive_scenarios), encoding="utf-8")

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
    print(f"benchmark_plot_3d_interactive={interactive_path}")
    print(f"benchmark_metrics_3d={csv_path}")


if __name__ == "__main__":
    main()
