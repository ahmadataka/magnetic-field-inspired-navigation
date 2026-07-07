#!/usr/bin/env python3
from __future__ import annotations

import csv
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import subprocess
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
    PrismObstacle,
    SphereObstacle,
    compute_metrics,
    make_default_scenarios_3d,
)


ARTIFACTS = ROOT / "artifacts"
WORLD_DIR = ROOT / "webots" / "worlds" / "generated_crazyflie_benchmarks_3d"
METRICS_CSV = ARTIFACTS / "benchmark_metrics_webots_crazyflie_3d.csv"
PLOT_PNG = ARTIFACTS / "benchmark_comparison_webots_crazyflie_3d.png"
PLOT_HTML = ARTIFACTS / "benchmark_comparison_webots_crazyflie_3d.html"
SCENARIO_JSON_ENV = "MFINAV_WEBOTS_SCENARIO_JSON"
METHOD_ENV = "MFINAV_WEBOTS_METHOD"
CONFIG_JSON_ENV = "MFINAV_WEBOTS_CONFIG_JSON"
WEBOTS_TARGET_SPAN = 3.2
WEBOTS_MARGIN_XY = 0.55
WEBOTS_MIN_Z = 0.22
WEBOTS_VIEW_MARGIN_Z = 0.75
METHOD_SPECS = {
    "paper_pd_3d": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric_3d": {"label": "MFI-Geometric", "color": "#2ca02c"},
}
METHOD_BASE_CONFIGS = {
    "paper_pd_3d": {
        "c_field": 12.0,
        "c_perp": 18.0,
        "speed_limit": 0.40,
        "kp_goal": 0.08,
        "kp_goal_relaxed": 0.08,
        "kd_goal": 0.45,
        "max_acceleration": 2.2,
        "max_speed_norm": 0.55,
    },
    "paper_geometric_3d": {
        "c_field": 16.0,
        "c_perp": 18.0,
        "speed_limit": 0.34,
        "kp_goal": 0.10,
        "kp_goal_relaxed": 0.06,
        "kd_goal": 0.45,
        "kp_geom": 0.28,
        "max_acceleration": 2.2,
        "max_speed_norm": 0.52,
    },
}


def _ensure_generated_project_links() -> None:
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    controllers_link = WORLD_DIR / "controllers"
    target = ROOT / "webots" / "controllers"
    if controllers_link.exists() or controllers_link.is_symlink():
        return
    controllers_link.symlink_to(target)


def _scenario_extents_3d(scenario) -> tuple[float, float, float, float, float, float]:
    xs = [float(scenario.start[0]), float(scenario.goal[0])]
    ys = [float(scenario.start[1]), float(scenario.goal[1])]
    zs = [float(scenario.start[2]), float(scenario.goal[2])]
    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, SphereObstacle):
            xs.extend([float(obstacle.center[0] - obstacle.radius), float(obstacle.center[0] + obstacle.radius)])
            ys.extend([float(obstacle.center[1] - obstacle.radius), float(obstacle.center[1] + obstacle.radius)])
            zs.extend([float(obstacle.center[2] - obstacle.radius), float(obstacle.center[2] + obstacle.radius)])
        elif isinstance(obstacle, PrismObstacle):
            xs.extend(float(vertex[0]) for vertex in obstacle.vertices_xy)
            ys.extend(float(vertex[1]) for vertex in obstacle.vertices_xy)
            zs.extend([float(obstacle.z_min), float(obstacle.z_max)])
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)


def _transform_scenario_for_webots(scenario):
    x_min, x_max, y_min, y_max, z_min, z_max = _scenario_extents_3d(scenario)
    dominant_span = max(x_max - x_min, y_max - y_min, z_max - z_min, 1e-9)
    scale = min(1.0, WEBOTS_TARGET_SPAN / dominant_span)
    z_shift = WEBOTS_MIN_Z - z_min * scale

    transformed_obstacles = []
    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, SphereObstacle):
            transformed_obstacles.append(
                SphereObstacle(center=obstacle.center * scale + np.array([0.0, 0.0, z_shift], dtype=float), radius=obstacle.radius * scale)
            )
        elif isinstance(obstacle, PrismObstacle):
            transformed_obstacles.append(
                PrismObstacle(
                    vertices_xy=obstacle.vertices_xy * scale,
                    z_min=obstacle.z_min * scale + z_shift,
                    z_max=obstacle.z_max * scale + z_shift,
                )
            )
        else:
            raise ValueError(f"Unsupported obstacle type in 3D Webots transform: {type(obstacle)!r}")

    transformed = replace(
        scenario,
        start=scenario.start * scale + np.array([0.0, 0.0, z_shift], dtype=float),
        goal=scenario.goal * scale + np.array([0.0, 0.0, z_shift], dtype=float),
        obstacles=type(scenario.obstacles)(obstacles=transformed_obstacles),
        description=f"{scenario.description} Uniformly scaled by {scale:.3f} and lifted by {z_shift:.3f} m for Webots Crazyflie.",
    )
    return transformed, scale


def _scenario_to_payload(scenario) -> dict[str, object]:
    obstacles: list[dict[str, object]] = []
    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, SphereObstacle):
            obstacles.append(
                {
                    "kind": "sphere",
                    "center": [float(obstacle.center[0]), float(obstacle.center[1]), float(obstacle.center[2])],
                    "radius": float(obstacle.radius),
                }
            )
        elif isinstance(obstacle, PrismObstacle):
            obstacles.append(
                {
                    "kind": "prism",
                    "vertices_xy": [[float(vertex[0]), float(vertex[1])] for vertex in obstacle.vertices_xy],
                    "z_min": float(obstacle.z_min),
                    "z_max": float(obstacle.z_max),
                }
            )
    return {
        "name": scenario.name,
        "description": scenario.description,
        "start": [float(scenario.start[0]), float(scenario.start[1]), float(scenario.start[2])],
        "goal": [float(scenario.goal[0]), float(scenario.goal[1]), float(scenario.goal[2])],
        "hover_height": float(scenario.start[2]),
        "obstacles": obstacles,
    }


def _triangulate_polygon(vertices: np.ndarray) -> list[tuple[int, int, int]]:
    remaining = list(range(len(vertices)))
    triangles: list[tuple[int, int, int]] = []

    def cross(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        ab = b - a
        ac = c - a
        return float(ab[0] * ac[1] - ab[1] * ac[0])

    def point_in_triangle(p: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
        c1 = cross(a, b, p)
        c2 = cross(b, c, p)
        c3 = cross(c, a, p)
        has_neg = c1 < -1e-9 or c2 < -1e-9 or c3 < -1e-9
        has_pos = c1 > 1e-9 or c2 > 1e-9 or c3 > 1e-9
        return not (has_neg and has_pos)

    orientation = 0.0
    for i in range(len(vertices)):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % len(vertices)]
        orientation += float(x1 * y2 - x2 * y1)
    sign = 1.0 if orientation >= 0.0 else -1.0

    guard = 0
    while len(remaining) > 3 and guard < 1000:
        guard += 1
        ear_found = False
        for idx in range(len(remaining)):
            i_prev = remaining[(idx - 1) % len(remaining)]
            i_curr = remaining[idx]
            i_next = remaining[(idx + 1) % len(remaining)]
            a = vertices[i_prev]
            b = vertices[i_curr]
            c = vertices[i_next]
            if sign * cross(a, b, c) <= 1e-9:
                continue
            if any(point_in_triangle(vertices[j], a, b, c) for j in remaining if j not in (i_prev, i_curr, i_next)):
                continue
            triangles.append((i_prev, i_curr, i_next))
            del remaining[idx]
            ear_found = True
            break
        if not ear_found:
            raise ValueError("Failed to triangulate polygon for Webots Crazyflie 3D world generation.")
    if len(remaining) == 3:
        triangles.append((remaining[0], remaining[1], remaining[2]))
    return triangles


def _prism_shape_block(prism: PrismObstacle) -> str:
    vertices = prism.vertices_xy
    triangles = _triangulate_polygon(vertices)
    n = len(vertices)

    points: list[str] = []
    for x, y in vertices:
        points.append(f"{float(x):.6f} {float(y):.6f} {float(prism.z_min):.6f}")
    for x, y in vertices:
        points.append(f"{float(x):.6f} {float(y):.6f} {float(prism.z_max):.6f}")

    faces: list[str] = []
    for a, b, c in triangles:
        faces.append(f"{a}, {b}, {c}, -1")
        faces.append(f"{c + n}, {b + n}, {a + n}, -1")
    for i in range(n):
        j = (i + 1) % n
        faces.append(f"{i}, {j}, {j + n}, -1")
        faces.append(f"{i}, {j + n}, {i + n}, -1")

    coord_points = ",\n              ".join(points)
    coord_index = "\n            ".join(faces)
    return f"""Shape {{
      appearance PBRAppearance {{
        baseColor 0.75 0.2 0.2
        roughness 0.55
      }}
      geometry IndexedFaceSet {{
        coord Coordinate {{
          point [
              {coord_points}
          ]
        }}
        coordIndex [
            {coord_index}
        ]
        creaseAngle 1.2
      }}
    }}"""


def _sphere_block(obstacle: SphereObstacle, index: int) -> str:
    return f"""DEF OBSTACLE_{index} Solid {{
  translation {float(obstacle.center[0]):.6f} {float(obstacle.center[1]):.6f} {float(obstacle.center[2]):.6f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.75 0.2 0.2
        roughness 0.55
      }}
      geometry Sphere {{
        radius {float(obstacle.radius):.6f}
      }}
    }}
  ]
  name "obstacle_{index}"
  boundingObject Sphere {{
    radius {float(obstacle.radius):.6f}
  }}
}}"""


def _prism_block(obstacle: PrismObstacle, index: int) -> str:
    shape = _prism_shape_block(obstacle)
    return f"""DEF OBSTACLE_{index} Solid {{
  children [
    {shape}
  ]
  name "obstacle_{index}"
  boundingObject Group {{
    children [
      {shape}
    ]
  }}
}}"""


def _world_bounds(scenario) -> tuple[float, float, float, float, float, float]:
    return _scenario_extents_3d(scenario)


def _world_text(scenario) -> str:
    x_min, x_max, y_min, y_max, z_min, z_max = _world_bounds(scenario)
    center_x = 0.5 * (x_min + x_max)
    center_y = 0.5 * (y_min + y_max)
    center_z = 0.5 * (z_min + z_max)
    span_x = x_max - x_min
    span_y = y_max - y_min
    span_z = z_max - z_min
    dominant = max(span_x, span_y, span_z, 1.0)
    viewpoint_x = center_x - 1.15 * dominant
    viewpoint_y = center_y - 0.85 * dominant
    viewpoint_z = center_z + 0.90 * dominant + WEBOTS_VIEW_MARGIN_Z

    obstacle_blocks: list[str] = []
    for index, obstacle in enumerate(scenario.obstacles.obstacles, start=1):
        if isinstance(obstacle, SphereObstacle):
            obstacle_blocks.append(_sphere_block(obstacle, index))
        elif isinstance(obstacle, PrismObstacle):
            obstacle_blocks.append(_prism_block(obstacle, index))

    floor_size = max(span_x, span_y, 3.5) + 1.0
    obstacles_text = "\n".join(obstacle_blocks)
    return f"""#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/bitcraze/crazyflie/protos/Crazyflie.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/Floor.proto"

WorldInfo {{
  title "MFI Webots Crazyflie 3D benchmark - {scenario.name}"
  basicTimeStep 32
}}
Viewpoint {{
  orientation -0.229 0.469 0.853 0.708
  position {viewpoint_x:.6f} {viewpoint_y:.6f} {viewpoint_z:.6f}
  follow "Crazyflie"
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
Floor {{
  size {floor_size:.6f} {floor_size:.6f}
}}
DEF GOAL Pose {{
  translation {float(scenario.goal[0]):.6f} {float(scenario.goal[1]):.6f} {float(scenario.goal[2]):.6f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0 0.8 0.2
        roughness 0.6
      }}
      geometry Sphere {{
        radius 0.06
      }}
    }}
  ]
}}
{obstacles_text}
Crazyflie {{
  translation {float(scenario.start[0]):.6f} {float(scenario.start[1]):.6f} {float(scenario.start[2]):.6f}
  rotation 0 0 1 0
  name "Crazyflie"
  controller "mfi_crazyflie_python"
  supervisor TRUE
}}"""


def _load_history(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return [{key: float(value) for key, value in row.items()} for row in rows]


def _plot_projection(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]], axes: tuple[str, str]) -> None:
    x_key, y_key = axes
    index_map = {"x": 0, "y": 1, "z": 2}
    for method_name, history in histories.items():
        xs = [row[x_key] for row in history]
        ys = [row[y_key] for row in history]
        ax.plot(xs, ys, linewidth=2.0, color=METHOD_SPECS[method_name]["color"], label=METHOD_SPECS[method_name]["label"])
        ax.scatter(xs[-1], ys[-1], color=METHOD_SPECS[method_name]["color"], s=55, marker="x")

    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, SphereObstacle):
            center = obstacle.center
            radius = obstacle.radius
            if axes == ("x", "y"):
                patch = plt.Circle((center[0], center[1]), radius, color="#d62728", alpha=0.16)
                ax.add_patch(patch)
            elif axes == ("x", "z"):
                patch = plt.Circle((center[0], center[2]), radius, color="#d62728", alpha=0.16)
                ax.add_patch(patch)
            elif axes == ("y", "z"):
                patch = plt.Circle((center[1], center[2]), radius, color="#d62728", alpha=0.16)
                ax.add_patch(patch)
        elif isinstance(obstacle, PrismObstacle):
            if axes == ("x", "y"):
                polygon = np.vstack((obstacle.vertices_xy, obstacle.vertices_xy[0]))
                ax.fill(polygon[:, 0], polygon[:, 1], color="#d62728", alpha=0.16)
                ax.plot(polygon[:, 0], polygon[:, 1], color="#d62728", linewidth=1.1, alpha=0.8)
            else:
                if axes == ("x", "z"):
                    min_h = float(np.min(obstacle.vertices_xy[:, 0]))
                    max_h = float(np.max(obstacle.vertices_xy[:, 0]))
                else:
                    min_h = float(np.min(obstacle.vertices_xy[:, 1]))
                    max_h = float(np.max(obstacle.vertices_xy[:, 1]))
                rect_x = [min_h, max_h, max_h, min_h, min_h]
                rect_y = [obstacle.z_min, obstacle.z_min, obstacle.z_max, obstacle.z_max, obstacle.z_min]
                ax.fill(rect_x, rect_y, color="#d62728", alpha=0.12)
                ax.plot(rect_x, rect_y, color="#d62728", linewidth=1.0, alpha=0.7)

    ax.scatter(scenario.start[index_map[x_key]], scenario.start[index_map[y_key]], color="#0f766e", s=60, marker="o", label="start")
    ax.scatter(scenario.goal[index_map[x_key]], scenario.goal[index_map[y_key]], color="#111111", s=90, marker="*", label="goal")
    ax.set_xlabel(x_key)
    ax.set_ylabel(y_key)
    ax.set_title(f"{scenario.name} {x_key}{y_key}")
    ax.grid(True, alpha=0.25)


def _downsample_history(history: list[dict[str, float]], max_points: int = 1000) -> list[dict[str, float]]:
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
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
        row_x: list[float] = []
        row_y: list[float] = []
        row_z: list[float] = []
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
    bottom = np.column_stack((vertices, np.full(len(vertices), prism.z_min)))
    top = np.column_stack((vertices, np.full(len(vertices), prism.z_max)))
    for loop in (bottom, top):
        closed = np.vstack((loop, loop[0]))
        traces.append(
            {
                "type": "scatter3d",
                "mode": "lines",
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
                "x": [float(vertex[0]), float(vertex[0])],
                "y": [float(vertex[1]), float(vertex[1])],
                "z": [float(prism.z_min), float(prism.z_max)],
                "line": {"color": "#ef4444", "width": 4},
                "hoverinfo": "skip",
                "showlegend": False,
                "meta": {"category": "context"},
            }
        )
    return traces


def _build_interactive_html(scenarios_data: list[dict[str, object]]) -> str:
    html_parts = [
        "<!DOCTYPE html>",
        "<html lang=\"en\">",
        "<head>",
        "  <meta charset=\"utf-8\">",
        "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">",
        "  <title>Webots Crazyflie 3D Benchmark</title>",
        "  <script src=\"https://cdn.plot.ly/plotly-2.35.2.min.js\"></script>",
        "  <style>",
        "    body { font-family: Helvetica, Arial, sans-serif; margin: 0; background: #f7f7f5; color: #1a1a1a; }",
        "    main { max-width: 1400px; margin: 0 auto; padding: 24px; }",
        "    h1 { margin: 0 0 8px; font-size: 28px; }",
        "    p { margin: 0 0 18px; line-height: 1.5; }",
        "    .scenario { background: #ffffff; border: 1px solid #dddddd; border-radius: 16px; padding: 18px; margin: 0 0 22px; box-shadow: 0 10px 24px rgba(0, 0, 0, 0.05); }",
        "    .plot { width: 100%; height: 720px; }",
        "    .controls { display: flex; gap: 10px; margin: 0 0 14px; flex-wrap: wrap; align-items: center; }",
        "    .controls button { border: 1px solid #c9c9c9; background: #f3f4f6; color: #222222; border-radius: 999px; padding: 8px 14px; cursor: pointer; font-size: 14px; }",
        "    .controls button.active { background: #111827; color: #ffffff; border-color: #111827; }",
        "  </style>",
        "</head>",
        "<body>",
        "  <main>",
        "    <h1>Webots Crazyflie 3D Benchmark</h1>",
        "    <p>These scenes preserve the quadrotor_3d geometry up to a uniform scale and a vertical lift so they fit the Webots Crazyflie workspace. Toggle algorithms per scene and drag to inspect the 3D trajectory.</p>",
    ]

    script_lines = [
        "<script>",
        "function flattenNumeric(values) {",
        "  if (!Array.isArray(values)) return typeof values === 'number' ? [values] : [];",
        "  const result = [];",
        "  const stack = [...values];",
        "  while (stack.length > 0) {",
        "    const value = stack.pop();",
        "    if (Array.isArray(value)) { for (let i = 0; i < value.length; i += 1) stack.push(value[i]); }",
        "    else if (typeof value === 'number' && Number.isFinite(value)) result.push(value);",
        "  }",
        "  return result;",
        "}",
        "function computeSceneRanges(traces, visibleMask) {",
        "  const xs = [], ys = [], zs = [];",
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
        "  return {'scene.xaxis.range': makeRange(xs), 'scene.yaxis.range': makeRange(ys), 'scene.zaxis.range': makeRange(zs), 'scene.aspectmode': 'data'};",
        "}",
        "function applySelection(plotId, traces, selectedAlgorithms) {",
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
        "function refreshSelection(plotId) {",
        "  const index = Number(plotId.split('-')[1]);",
        "  const traces = window[`data${index}`];",
        "  applySelection(plotId, traces, selectedAlgorithmsForPlot(plotId));",
        "}",
    ]

    for index, scenario_data in enumerate(scenarios_data):
        div_id = f"plot-{index}"
        html_parts.append("    <section class=\"scenario\">")
        html_parts.append(f"      <h2>{scenario_data['name']}</h2>")
        html_parts.append(f"      <p>{scenario_data['description']}</p>")
        html_parts.append("      <div class=\"controls\">")
        html_parts.append(f"        <button data-plot=\"{div_id}\" data-action=\"all\">All</button>")
        html_parts.append(f"        <button data-plot=\"{div_id}\" data-action=\"none\">None</button>")
        for method_name, spec in METHOD_SPECS.items():
            html_parts.append(f"        <button class=\"active\" data-role=\"toggle\" data-plot=\"{div_id}\" data-algorithm=\"{method_name}\">{spec['label']}</button>")
        html_parts.append("      </div>")
        html_parts.append(f"      <div id=\"{div_id}\" class=\"plot\"></div>")
        html_parts.append("    </section>")
        script_lines.append(f"const data{index} = {json.dumps(scenario_data['plot_traces'], separators=(',', ':'))};")
        script_lines.append(f"window.data{index} = data{index};")
        script_lines.append(f"const layout{index} = {json.dumps(scenario_data['layout'], separators=(',', ':'))};")
        script_lines.append(f"Plotly.newPlot('{div_id}', data{index}, layout{index}, {{responsive: true, displaylogo: false}});")
        script_lines.append(f"refreshSelection('{div_id}');")

    script_lines.extend(
        [
            "document.querySelectorAll('.controls button').forEach((button) => {",
            "  button.addEventListener('click', () => {",
            "    const plotId = button.dataset.plot;",
            "    if (button.dataset.action === 'all') {",
            "      document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((toggle) => toggle.classList.add('active'));",
            "      refreshSelection(plotId);",
            "      return;",
            "    }",
            "    if (button.dataset.action === 'none') {",
            "      document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"]`).forEach((toggle) => toggle.classList.remove('active'));",
            "      refreshSelection(plotId);",
            "      return;",
            "    }",
            "    if (button.dataset.role === 'toggle') {",
            "      const activeToggles = document.querySelectorAll(`[data-plot=\"${plotId}\"][data-role=\"toggle\"].active`).length;",
            "      const willDeactivate = button.classList.contains('active');",
            "      if (willDeactivate && activeToggles === 1) return;",
            "      button.classList.toggle('active');",
            "      refreshSelection(plotId);",
            "    }",
            "  });",
            "});",
            "</script>",
        ]
    )

    html_parts.extend(script_lines)
    html_parts.extend(["  </main>", "</body>", "</html>"])
    return "\n".join(html_parts)


def _method_overrides(scale: float, method_name: str) -> dict[str, float]:
    base = dict(METHOD_BASE_CONFIGS[method_name])
    base["r_l"] = 4.0 * scale
    base["r_la"] = 2.0 * scale
    base["sensor_range"] = 6.0 * scale
    return base


def _run_one_scenario(scenario, scale: float, method_name: str) -> tuple[list[dict[str, float]], dict[str, object], Path]:
    _ensure_generated_project_links()
    scenario_dir = WORLD_DIR / scenario.name
    scenario_dir.mkdir(parents=True, exist_ok=True)
    world_path = scenario_dir / f"{scenario.name}.wbt"
    history_path = scenario_dir / f"history_{method_name}.csv"
    summary_path = scenario_dir / f"summary_{method_name}.json"
    console_path = scenario_dir / f"console_{method_name}.log"
    world_path.write_text(_world_text(scenario))

    env = os.environ.copy()
    env["MFINAV_WEBOTS_HISTORY"] = str(history_path)
    env["MFINAV_WEBOTS_SUMMARY"] = str(summary_path)
    env["MFINAV_WEBOTS_MAX_STEPS"] = env.get("MFINAV_WEBOTS_MAX_STEPS", "2200")
    env["MFINAV_WEBOTS_QUIT"] = "1"
    env[METHOD_ENV] = method_name
    env[CONFIG_JSON_ENV] = json.dumps(_method_overrides(scale, method_name), separators=(",", ":"))
    env[SCENARIO_JSON_ENV] = json.dumps(_scenario_to_payload(scenario), separators=(",", ":"))

    command = [
        "/Applications/Webots.app/Contents/MacOS/webots",
        "--batch",
        "--mode=fast",
        "--no-rendering",
        "--stdout",
        "--stderr",
        str(world_path),
    ]
    result = subprocess.run(
        command,
        cwd=str(ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    console_path.write_text(result.stdout)
    if not history_path.exists():
        raise RuntimeError(f"Webots Crazyflie 3D did not produce a history for scenario {scenario.name} ({method_name}). See {console_path}.")
    history = _load_history(history_path)
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {"status": "unknown"}
    summary["webots_exit_code"] = result.returncode
    summary_path.write_text(json.dumps(summary, indent=2))
    return history, summary, world_path


def main() -> None:
    transformed_scenarios = [_transform_scenario_for_webots(scenario) for scenario in make_default_scenarios_3d()]
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    summary_rows: list[dict[str, str | float]] = []
    interactive_scenarios: list[dict[str, object]] = []

    fig, axes = plt.subplots(len(transformed_scenarios), 3, figsize=(16, 4.6 * len(transformed_scenarios)))
    axes = np.atleast_2d(axes)

    for row_axes, (scenario, scale) in zip(axes, transformed_scenarios):
        histories: dict[str, list[dict[str, float]]] = {}
        plot_traces: list[dict[str, object]] = []

        for obstacle in scenario.obstacles.obstacles:
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
                        "colorscale": [[0.0, "#ef4444"], [1.0, "#ef4444"]],
                        "meta": {"category": "context"},
                    }
                )
            elif isinstance(obstacle, PrismObstacle):
                plot_traces.extend(_prism_wireframe(obstacle))

        for method_name in METHOD_SPECS:
            history, summary, world_path = _run_one_scenario(scenario, scale, method_name)
            histories[method_name] = history
            reduced_history = _downsample_history(history)
            plot_traces.append(
                {
                    "type": "scatter3d",
                    "mode": "lines",
                    "name": METHOD_SPECS[method_name]["label"],
                    "x": [row["x"] for row in reduced_history],
                    "y": [row["y"] for row in reduced_history],
                    "z": [row["z"] for row in reduced_history],
                    "line": {"color": METHOD_SPECS[method_name]["color"], "width": 7 if method_name == "paper_pd_3d" else 6},
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
                    "marker": {"size": 4, "symbol": "x", "color": METHOD_SPECS[method_name]["color"]},
                    "meta": {"category": "algorithm", "algorithm": method_name},
                    "showlegend": False,
                }
            )

            metrics = summary.get("metrics", {})
            summary_rows.append(
                {
                    "scenario": scenario.name,
                    "method": method_name,
                    "success": float(metrics.get("success", 0.0)),
                    "goal_reached_once": float(metrics.get("goal_reached_once", 0.0)),
                    "steps": float(metrics.get("steps", len(history))),
                    "path_length": float(metrics.get("path_length", 0.0)),
                    "final_goal_distance": float(metrics.get("final_goal_distance", history[-1]["goal_distance"])),
                    "min_clearance": float(metrics.get("min_clearance", math.inf)),
                    "mean_speed": float(metrics.get("mean_speed", 0.0)),
                    "collision": float(metrics.get("collision", 0.0)),
                    "safety_violation": float(metrics.get("safety_violation", 0.0)),
                    "time_to_goal_steps": float(metrics.get("time_to_goal_steps", math.inf)),
                    "path_efficiency": float(metrics.get("path_efficiency", 0.0)),
                    "status": str(summary.get("status", "unknown")),
                    "webots_exit_code": float(summary.get("webots_exit_code", -1)),
                    "world_path": str(world_path),
                    "scale": float(scale),
                }
            )

        for ax, projection in zip(row_axes, (("x", "y"), ("x", "z"), ("y", "z"))):
            _plot_projection(ax, scenario, histories, projection)

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
                        "camera": {"eye": {"x": 1.6, "y": 1.4, "z": 0.95}},
                    },
                    "title": {"text": scenario.name.replace("_", " ")},
                },
            }
        )

    for axis_row in axes:
        axis_row[0].legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(PLOT_PNG, dpi=180)
    plt.close(fig)

    PLOT_HTML.write_text(_build_interactive_html(interactive_scenarios), encoding="utf-8")

    with METRICS_CSV.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)

    for row in summary_rows:
        print(
            f"{row['scenario']} {row['method']}: "
            f"status={row['status']} "
            f"success={int(row['success'])} "
            f"collision={int(row['collision'])} "
            f"safety_violation={int(row['safety_violation'])} "
            f"final_goal_distance={row['final_goal_distance']:.3f} "
            f"min_clearance={row['min_clearance']:.3f}"
        )

    print(f"benchmark_plot_webots_crazyflie_3d={PLOT_PNG}")
    print(f"benchmark_html_webots_crazyflie_3d={PLOT_HTML}")
    print(f"benchmark_metrics_webots_crazyflie_3d={METRICS_CSV}")


if __name__ == "__main__":
    main()
