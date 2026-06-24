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
    DoubleIntegratorState,
    HaddadinNavigator,
    MagneticFieldNavigator3D,
    PrismObstacle,
    SabattiniNavigator,
    SphereObstacle,
    compute_metrics,
    make_dynamic_scenarios_3d,
    make_paper_geometric_3d_config,
    make_paper_pd_3d_config,
    simulate,
)

METHOD_SPECS = {
    "paper_pd_3d": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric_3d": {"label": "MFI-Geometric", "color": "#2ca02c"},
    "apf_3d": {"label": "APF", "color": "#ff7f0e"},
    "haddadin_3d": {"label": "Haddadin", "color": "#8b5cf6"},
    "sabattini_3d": {"label": "Sabattini", "color": "#d97706"},
}


def _initial_state(start: np.ndarray) -> DoubleIntegratorState:
    return DoubleIntegratorState(position=start.copy(), velocity=np.zeros_like(start))


def _downsample_history(history: list[dict[str, float]], max_points: int = 450) -> list[dict[str, float]]:
    if len(history) <= max_points:
        return history
    indices = np.linspace(0, len(history) - 1, max_points, dtype=int)
    return [history[index] for index in indices]


def _sample_indices(max_len: int, target_frames: int = 34) -> list[int]:
    if max_len <= 1:
        return [0]
    count = min(target_frames, max_len)
    return sorted({int(round(value)) for value in np.linspace(0, max_len - 1, count)})


def _sphere_wireframe(shape: dict[str, object], samples: int = 28) -> tuple[list[float], list[float], list[float]]:
    cx, cy, cz = shape["center"]
    radius = float(shape["radius"])
    angles = np.linspace(0.0, 2.0 * math.pi, samples)
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    for plane in ("xy", "xz", "yz"):
        for angle in angles:
            if plane == "xy":
                xs.append(cx + radius * math.cos(angle))
                ys.append(cy + radius * math.sin(angle))
                zs.append(cz)
            elif plane == "xz":
                xs.append(cx + radius * math.cos(angle))
                ys.append(cy)
                zs.append(cz + radius * math.sin(angle))
            else:
                xs.append(cx)
                ys.append(cy + radius * math.cos(angle))
                zs.append(cz + radius * math.sin(angle))
        xs.append(None)
        ys.append(None)
        zs.append(None)
    return xs, ys, zs


def _prism_wireframe_from_shape(shape: dict[str, object]) -> tuple[list[float], list[float], list[float]]:
    vertices = np.asarray(shape["vertices_xy"], dtype=float)
    z_min = float(shape["z_min"])
    z_max = float(shape["z_max"])
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    loops = [
        np.column_stack((vertices, np.full(len(vertices), z_min))),
        np.column_stack((vertices, np.full(len(vertices), z_max))),
    ]
    for loop in loops:
        closed = np.vstack((loop, loop[0]))
        xs.extend(closed[:, 0].tolist() + [None])
        ys.extend(closed[:, 1].tolist() + [None])
        zs.extend(closed[:, 2].tolist() + [None])
    for vertex in vertices:
        xs.extend([float(vertex[0]), float(vertex[0]), None])
        ys.extend([float(vertex[1]), float(vertex[1]), None])
        zs.extend([z_min, z_max, None])
    return xs, ys, zs


def _shape_to_wireframe(shape: dict[str, object]) -> tuple[list[float], list[float], list[float]]:
    if shape["kind"] == "sphere":
        return _sphere_wireframe(shape)
    if shape["kind"] == "prism":
        return _prism_wireframe_from_shape(shape)
    raise ValueError(f"Unsupported shape kind {shape['kind']}.")


def _make_range(values: list[float]) -> list[float]:
    min_v, max_v = min(values), max(values)
    span = max(max_v - min_v, 1.0)
    pad = 0.15 * span
    return [min_v - pad, max_v + pad]


def _plot_prism_projection(ax: plt.Axes, shape: dict[str, object], axes: tuple[str, str]) -> None:
    x_key, y_key = axes
    axis_map = {"x": 0, "y": 1, "z": 2}
    vertices_xy = np.asarray(shape["vertices_xy"], dtype=float)
    z_min = float(shape["z_min"])
    z_max = float(shape["z_max"])
    if axes == ("x", "y"):
        polygon = np.vstack((vertices_xy, vertices_xy[0]))
        ax.fill(polygon[:, 0], polygon[:, 1], color="#ef4444", alpha=0.18)
        ax.plot(polygon[:, 0], polygon[:, 1], color="#ef4444", linewidth=1.2, alpha=0.8)
        return
    min_x = float(np.min(vertices_xy[:, axis_map[x_key]])) if x_key != "z" else z_min
    max_x = float(np.max(vertices_xy[:, axis_map[x_key]])) if x_key != "z" else z_max
    min_y = float(np.min(vertices_xy[:, axis_map[y_key]])) if y_key != "z" else z_min
    max_y = float(np.max(vertices_xy[:, axis_map[y_key]])) if y_key != "z" else z_max
    rect_x = [min_x, max_x, max_x, min_x, min_x]
    rect_y = [min_y, min_y, max_y, max_y, min_y]
    ax.fill(rect_x, rect_y, color="#ef4444", alpha=0.12)
    ax.plot(rect_x, rect_y, color="#ef4444", linewidth=1.0, alpha=0.7)


def _plot_projection(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]], axes: tuple[str, str], snapshot: dict[str, object]) -> None:
    axis_map = {"x": 0, "y": 1, "z": 2}
    x_key, y_key = axes
    for name, history in histories.items():
        xs = [row[x_key] for row in history]
        ys = [row[y_key] for row in history]
        color = METHOD_SPECS[name]["color"]
        ax.plot(xs, ys, linewidth=2.0, label=METHOD_SPECS[name]["label"], color=color)
        ax.scatter(xs[-1], ys[-1], color=color, s=42, marker="x")
    for shape in snapshot["obstacles"]:
        if shape["kind"] == "sphere":
            center = shape["center"]
            ax.scatter(center[axis_map[x_key]], center[axis_map[y_key]], color="#d62728", s=50, alpha=0.4)
        elif shape["kind"] == "prism":
            _plot_prism_projection(ax, shape, axes)
    ax.scatter(scenario.start[axis_map[x_key]], scenario.start[axis_map[y_key]], color="#0f766e", s=55, marker="o")
    ax.scatter(scenario.goal[axis_map[x_key]], scenario.goal[axis_map[y_key]], color="#111111", s=85, marker="*")
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
        "  <title>Dynamic 3D Benchmark Comparison</title>",
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
        "    .metrics { width: 100%; border-collapse: collapse; margin-top: 14px; font-size: 14px; }",
        "    .metrics th, .metrics td { border-bottom: 1px solid #e5e7eb; padding: 8px 10px; text-align: right; }",
        "    .metrics th:first-child, .metrics td:first-child { text-align: left; }",
        "    .metrics th { background: #f9fafb; }",
        "  </style>",
        "</head>",
        "<body><main>",
        "    <h1>Interactive Dynamic 3D Benchmark Comparison</h1>",
        "    <p>Each scene animates moving obstacle geometry together with the robot trajectories. Use Play/Pause to inspect obstacle motion and toggle algorithms to compare specific methods.</p>",
    ]
    script_lines = [
        "<script>",
        "function flattenNumeric(values) { if (!Array.isArray(values)) return typeof values === 'number' ? [values] : []; const result = []; const stack = [...values]; while (stack.length > 0) { const value = stack.pop(); if (Array.isArray(value)) { for (let i = 0; i < value.length; i += 1) stack.push(value[i]); } else if (typeof value === 'number' && Number.isFinite(value)) result.push(value); } return result; }",
        "function computeSceneRanges(traces, visibleMask) { const xs = [], ys = [], zs = []; traces.forEach((trace, index) => { if (!visibleMask[index]) return; xs.push(...flattenNumeric(trace.x || [])); ys.push(...flattenNumeric(trace.y || [])); zs.push(...flattenNumeric(trace.z || [])); }); if (!xs.length || !ys.length || !zs.length) return null; const makeRange = (values) => { const min = Math.min(...values); const max = Math.max(...values); const span = Math.max(max - min, 1.0); const pad = 0.14 * span; return [min - pad, max + pad]; }; return {'scene.xaxis.range': makeRange(xs), 'scene.yaxis.range': makeRange(ys), 'scene.zaxis.range': makeRange(zs), 'scene.aspectmode': 'data'}; }",
        "function selectedAlgorithmsForPlot(plotId) { return Array.from(document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"].active`)).map((button) => button.dataset.algorithm); }",
        "function applyAlgorithmSelection(plotId, traces) { const selectedAlgorithms = selectedAlgorithmsForPlot(plotId); const visibleMask = traces.map((trace) => { const meta = trace.meta || {}; if (meta.category !== 'algorithm') return true; return selectedAlgorithms.includes(meta.algorithm); }); Plotly.restyle(plotId, {visible: visibleMask}); const ranges = computeSceneRanges(traces, visibleMask); if (ranges) Plotly.relayout(plotId, ranges); document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((button) => { button.classList.toggle('active', selectedAlgorithms.includes(button.dataset.algorithm)); }); }",
        "function refreshPlotSelection(plotId) { const index = Number(plotId.split('-')[1]); applyAlgorithmSelection(plotId, window[`data${index}`]); }",
    ]
    for index, scenario_data in enumerate(scenarios_data):
        div_id = f"plot-{index}"
        html_parts.append(f"<section class=\"scenario\"><h2>{scenario_data['name']}</h2><p class=\"caption\">{scenario_data['description']}</p><div class=\"controls\">")
        html_parts.append(f"<button class=\"utility\" data-plot=\"{div_id}\" data-action=\"play\">Play</button>")
        html_parts.append(f"<button class=\"utility\" data-plot=\"{div_id}\" data-action=\"pause\">Pause</button>")
        html_parts.append(f"<button class=\"utility\" data-plot=\"{div_id}\" data-action=\"all\">All</button>")
        html_parts.append(f"<button class=\"utility\" data-plot=\"{div_id}\" data-action=\"none\">None</button>")
        for method_name, spec in METHOD_SPECS.items():
            html_parts.append(f"<button class=\"active\" data-role=\"toggle\" data-plot=\"{div_id}\" data-algorithm=\"{method_name}\">{spec['label']}</button>")
        html_parts.append(f"</div><div id=\"{div_id}\" class=\"plot\"></div>{scenario_data['metrics_table']}</section>")
        script_lines.append(f"const data{index} = {json.dumps(scenario_data['plot_traces'], separators=(',', ':'))};")
        script_lines.append(f"window.data{index} = data{index};")
        script_lines.append(f"const layout{index} = {json.dumps(scenario_data['layout'], separators=(',', ':'))};")
        script_lines.append(f"const frames{index} = {json.dumps(scenario_data['frames'], separators=(',', ':'))};")
        script_lines.append(f"Plotly.newPlot('{div_id}', data{index}, layout{index}, {{responsive: true, displaylogo: false}}).then(() => {{ Plotly.addFrames('{div_id}', frames{index}); refreshPlotSelection('{div_id}'); }});")
    script_lines.append("document.querySelectorAll('.controls button').forEach((button) => { button.addEventListener('click', () => { const plotId = button.dataset.plot; if (button.dataset.action === 'play') { Plotly.animate(plotId, null, {fromcurrent: true, frame: {duration: 110, redraw: true}, transition: {duration: 0}}); return; } if (button.dataset.action === 'pause') { Plotly.animate(plotId, [[null]], {mode: 'immediate', frame: {duration: 0, redraw: false}, transition: {duration: 0}}); return; } if (button.dataset.action === 'all') { document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((toggle) => toggle.classList.add('active')); refreshPlotSelection(plotId); return; } if (button.dataset.action === 'none') { document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((toggle) => toggle.classList.remove('active')); refreshPlotSelection(plotId); return; } if (button.dataset.role === 'toggle') { const activeCount = document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"].active`).length; const willDeactivate = button.classList.contains('active'); if (willDeactivate && activeCount === 1) return; button.classList.toggle('active'); refreshPlotSelection(plotId); } }); });")
    script_lines.append("</script>")
    html_parts.extend(script_lines)
    html_parts.extend(["</main></body></html>"])
    return "\n".join(html_parts)


def _metrics_table_html(metrics_rows: list[dict[str, str | float]]) -> str:
    lines = ["<table class=\"metrics\"><thead><tr><th>Method</th><th>Success</th><th>Reach Once</th><th>Final Error</th><th>Min Clearance</th><th>Collision</th><th>Safe Viol</th><th>Path Length</th></tr></thead><tbody>"]
    for row in metrics_rows:
        lines.append(
            "<tr>"
            f"<td>{row['method']}</td><td>{int(row['success'])}</td><td>{int(row['goal_reached_once'])}</td>"
            f"<td>{row['final_goal_distance']:.3f}</td><td>{row['min_clearance']:.3f}</td><td>{int(row['collision'])}</td>"
            f"<td>{int(row['safety_violation'])}</td><td>{row['path_length']:.3f}</td></tr>"
        )
    lines.append("</tbody></table>")
    return "".join(lines)


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
            "paper_pd_3d": simulate(_initial_state(scenario.start), scenario.goal, scenario.obstacles, config_pd, navigator=MagneticFieldNavigator3D(config_pd)),
            "paper_geometric_3d": simulate(_initial_state(scenario.start), scenario.goal, scenario.obstacles, config_geometric, navigator=MagneticFieldNavigator3D(config_geometric)),
            "apf_3d": simulate(_initial_state(scenario.start), scenario.goal, scenario.obstacles, config_pd, navigator=ArtificialPotentialFieldNavigator(config_pd)),
            "haddadin_3d": simulate(_initial_state(scenario.start), scenario.goal, scenario.obstacles, config_pd, navigator=HaddadinNavigator(config_pd)),
            "sabattini_3d": simulate(_initial_state(scenario.start), scenario.goal, scenario.obstacles, config_pd, navigator=SabattiniNavigator(config_pd)),
        }
        max_len = max(len(history) for history in histories.values())
        frame_indices = _sample_indices(max_len, target_frames=30)
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
            plot_traces.append(
                {
                    "type": "scatter3d",
                    "mode": "lines",
                    "x": xs,
                    "y": ys,
                    "z": zs,
                    "line": {"color": "#ef4444", "width": 5},
                    "name": f"Obstacle {obstacle_index + 1}",
                    "hoverinfo": "skip",
                    "meta": {"category": "context"},
                    "showlegend": obstacle_index == 0,
                }
            )
        downsampled_histories = {name: _downsample_history(history) for name, history in histories.items()}
        for method_name, history in downsampled_histories.items():
            spec = METHOD_SPECS[method_name]
            x0 = [history[0]["x"]]
            y0 = [history[0]["y"]]
            z0 = [history[0]["z"]]
            plot_traces.append(
                {
                    "type": "scatter3d",
                    "mode": "lines",
                    "x": x0,
                    "y": y0,
                    "z": z0,
                    "line": {"color": spec["color"], "width": 6},
                    "name": spec["label"],
                    "meta": {"category": "algorithm", "algorithm": method_name},
                }
            )
            plot_traces.append(
                {
                    "type": "scatter3d",
                    "mode": "markers",
                    "x": x0,
                    "y": y0,
                    "z": z0,
                    "marker": {"color": spec["color"], "size": 4, "symbol": "x"},
                    "name": f"{spec['label']} current",
                    "showlegend": False,
                    "hoverinfo": "skip",
                    "meta": {"category": "algorithm", "algorithm": method_name},
                }
            )
            metrics = compute_metrics(history, scenario.goal)
            row = {
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
        interactive_scenarios.append(
            {
                "name": scenario.name.replace("_", " "),
                "description": scenario.description,
                "plot_traces": plot_traces,
                "frames": frames,
                "metrics_table": _metrics_table_html(metrics_rows_for_html),
                "layout": {
                    "title": {"text": f"{scenario.name.replace('_', ' ')} - t = 0.00 s"},
                    "scene": {
                        "xaxis": {"title": "x", "range": _make_range(all_x)},
                        "yaxis": {"title": "y", "range": _make_range(all_y)},
                        "zaxis": {"title": "z", "range": _make_range(all_z)},
                        "aspectmode": "data",
                    },
                    "legend": {"orientation": "h", "yanchor": "bottom", "y": 1.02, "x": 0.0},
                    "margin": {"l": 0, "r": 0, "t": 70, "b": 0},
                    "sliders": [{"active": 0, "currentvalue": {"prefix": "Frame: "}, "pad": {"t": 18}, "steps": [{"label": str(frame_id), "method": "animate", "args": [[f"frame-{frame_id}"], {"mode": "immediate", "frame": {"duration": 0, "redraw": True}, "transition": {"duration": 0}}]} for frame_id in range(len(frames))]}],
                },
            }
        )

    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=5, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    plot_path = artifacts / "benchmark_comparison_dynamic_3d.png"
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    csv_path = artifacts / "benchmark_metrics_dynamic_3d.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["scenario", "method", "success", "goal_reached_once", "steps", "path_length", "final_goal_distance", "min_clearance", "mean_speed", "collision", "safety_violation", "time_to_goal_steps", "path_efficiency"])
        writer.writeheader()
        writer.writerows(summary_rows)
    html_path = artifacts / "benchmark_comparison_dynamic_3d.html"
    html_path.write_text(_build_interactive_html(interactive_scenarios), encoding="utf-8")
    print(f"benchmark_plot_dynamic_3d={plot_path}")
    print(f"benchmark_metrics_dynamic_3d={csv_path}")
    print(f"benchmark_html_dynamic_3d={html_path}")


if __name__ == "__main__":
    main()
