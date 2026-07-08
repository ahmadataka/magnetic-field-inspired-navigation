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
from matplotlib.patches import Polygon as PolygonPatch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mfinav import PolygonObstacle, make_default_scenarios  # noqa: E402
from mfinav.utils.paths import benchmark_artifact_dir, generated_world_dir  # noqa: E402


ARTIFACTS = benchmark_artifact_dir(ROOT, "webots_epuck_static")
WORLD_DIR = generated_world_dir(ROOT, "epuck", "static")
METRICS_CSV = ARTIFACTS / "benchmark_metrics_webots_epuck.csv"
PLOT_PNG = ARTIFACTS / "benchmark_comparison_webots_epuck.png"
SCENARIO_JSON_ENV = "MFINAV_WEBOTS_SCENARIO_JSON"
METHOD_ENV = "MFINAV_WEBOTS_METHOD"
GOAL_TOLERANCE_ENV = "MFINAV_WEBOTS_GOAL_TOLERANCE"
CONFIG_JSON_ENV = "MFINAV_WEBOTS_CONFIG_JSON"
WEBOTS_TARGET_MAX_SPAN = 4.5
WEBOTS_MARGIN = 1.0
METHOD_SPECS = {
    "paper_pd": {"label": "PAPER_PD", "color": "#1f77b4"},
    "paper_geometric": {"label": "PAPER_GEOMETRIC", "color": "#2ca02c"},
    "apf": {"label": "APF", "color": "#ff7f0e"},
    "haddadin": {"label": "HADDADIN", "color": "#8b5cf6"},
    "sabattini": {"label": "SABATTINI", "color": "#d97706"},
}
METHOD_CONFIG_OVERRIDES = {
    "paper_pd": {
        "kp_goal": 0.16,
        "kp_goal_relaxed": 0.16,
        "kd_goal": 0.3,
        "c_field": 5.0,
        "c_perp": 8.0,
        "r_l": 2.5,
        "r_la": 1.0,
    },
    "paper_geometric": {
        "kp_goal_relaxed": 0.12,
        "kp_geom": 5.0,
        "c_field": 5.0,
        "c_perp": 8.0,
        "speed_limit": 0.8,
        "r_l": 2.5,
        "r_la": 1.0,
    },
    "apf": {},
    "haddadin": {},
    "sabattini": {},
}


def _ensure_generated_project_links() -> None:
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    controllers_link = WORLD_DIR / "controllers"
    target = ROOT / "webots" / "controllers"
    if controllers_link.exists() or controllers_link.is_symlink():
        return
    controllers_link.symlink_to(target)


def _scenario_to_payload(scenario) -> dict[str, object]:
    heading = math.atan2(float(scenario.goal[1] - scenario.start[1]), float(scenario.goal[0] - scenario.start[0]))
    obstacles = []
    for obstacle in scenario.obstacles.obstacles:
        if not isinstance(obstacle, PolygonObstacle):
            raise ValueError(f"Webots e-puck benchmark currently supports polygon obstacles only: {scenario.name}")
        obstacles.append(
            {
                "vertices": [[float(vertex[0]), float(vertex[1])] for vertex in obstacle.vertices],
            }
        )
    return {
        "name": scenario.name,
        "description": scenario.description,
        "start": [float(scenario.start[0]), float(scenario.start[1])],
        "goal": [float(scenario.goal[0]), float(scenario.goal[1])],
        "heading": heading,
        "obstacles": obstacles,
    }


def _rescale_scenario_for_webots(scenario):
    xs = [float(scenario.start[0]), float(scenario.goal[0])]
    ys = [float(scenario.start[1]), float(scenario.goal[1])]
    for obstacle in scenario.obstacles.obstacles:
        xs.extend(float(vertex[0]) for vertex in obstacle.vertices)
        ys.extend(float(vertex[1]) for vertex in obstacle.vertices)
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    dominant_span = max(span_x, span_y, 1e-9)
    scale = min(1.0, WEBOTS_TARGET_MAX_SPAN / dominant_span)
    if abs(scale - 1.0) < 1e-9:
        return scenario

    scaled_obstacles = []
    for obstacle in scenario.obstacles.obstacles:
        scaled_obstacles.append(PolygonObstacle(vertices=np.asarray(obstacle.vertices, dtype=float) * scale))
    return replace(
        scenario,
        start=np.asarray(scenario.start, dtype=float) * scale,
        goal=np.asarray(scenario.goal, dtype=float) * scale,
        obstacles=type(scenario.obstacles)(obstacles=scaled_obstacles),
        description=f"{scenario.description} Rescaled by {scale:.3f} for Webots e-puck arena.",
    )


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
            if any(
                point_in_triangle(vertices[j], a, b, c)
                for j in remaining
                if j not in (i_prev, i_curr, i_next)
            ):
                continue
            triangles.append((i_prev, i_curr, i_next))
            del remaining[idx]
            ear_found = True
            break
        if not ear_found:
            raise ValueError("Failed to triangulate polygon for Webots world generation.")
    if len(remaining) == 3:
        triangles.append((remaining[0], remaining[1], remaining[2]))
    return triangles


def _obstacle_shape_block(vertices: np.ndarray, height: float = 0.12) -> str:
    triangles = _triangulate_polygon(vertices)
    points: list[str] = []
    n = len(vertices)
    for x, y in vertices:
        points.append(f"{float(x):.6f} {float(y):.6f} 0")
    for x, y in vertices:
        points.append(f"{float(x):.6f} {float(y):.6f} {height:.6f}")

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


def _bounds_for_scenario(scenario) -> tuple[float, float, float, float]:
    xs = [float(scenario.start[0]), float(scenario.goal[0])]
    ys = [float(scenario.start[1]), float(scenario.goal[1])]
    for obstacle in scenario.obstacles.obstacles:
        xs.extend(float(vertex[0]) for vertex in obstacle.vertices)
        ys.extend(float(vertex[1]) for vertex in obstacle.vertices)
    margin = WEBOTS_MARGIN
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def _world_text(scenario) -> str:
    x_min, x_max, y_min, y_max = _bounds_for_scenario(scenario)
    center_x = 0.5 * (x_min + x_max)
    center_y = 0.5 * (y_min + y_max)
    size_x = x_max - x_min
    size_y = y_max - y_min
    viewpoint_x = center_x
    viewpoint_y = center_y - 0.15 * size_y
    viewpoint_z = max(size_x, size_y) * 1.2
    robot_heading = math.atan2(float(scenario.goal[1] - scenario.start[1]), float(scenario.goal[0] - scenario.start[0]))

    obstacle_blocks: list[str] = []
    for index, obstacle in enumerate(scenario.obstacles.obstacles):
        shape_block = _obstacle_shape_block(obstacle.vertices)
        obstacle_blocks.append(
            f"""DEF OBSTACLE_{index + 1} Solid {{
  children [
    {shape_block}
  ]
  name "obstacle_{index + 1}"
  boundingObject Group {{
    children [
      {shape_block}
    ]
  }}
}}"""
        )

    obstacles_text = "\n".join(obstacle_blocks)
    return f"""#VRML_SIM R2023b utf8

WorldInfo {{
  title "MFI Webots benchmark - {scenario.name}"
  basicTimeStep 64
}}
Viewpoint {{
  orientation -0.5213170125513537 -0.550708046966413 0.6518812924382558 4.218704350013546
  position {viewpoint_x:.6f} {viewpoint_y:.6f} {viewpoint_z:.6f}
  follow "mfi_robot"
}}
Background {{
  skyColor [
    0.92 0.94 0.98
  ]
}}
DirectionalLight {{
  ambientIntensity 0.8
  direction -0.3 -0.4 -1
}}
DEF FLOOR Solid {{
  translation {center_x:.6f} {center_y:.6f} -0.02
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.2 0.2 0.2
        roughness 1
        metalness 0
      }}
      geometry Box {{
        size {size_x:.6f} {size_y:.6f} 0.04
      }}
    }}
  ]
  name "floor"
  boundingObject Box {{
    size {size_x:.6f} {size_y:.6f} 0.04
  }}
}}
DEF WALL_N Solid {{
  translation {center_x:.6f} {y_max:.6f} 0.1
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.55 0.55 0.6
      }}
      geometry Box {{
        size {size_x:.6f} 0.06 0.2
      }}
    }}
  ]
  name "wall_n"
  boundingObject Box {{
    size {size_x:.6f} 0.06 0.2
  }}
}}
DEF WALL_S Solid {{
  translation {center_x:.6f} {y_min:.6f} 0.1
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.55 0.55 0.6
      }}
      geometry Box {{
        size {size_x:.6f} 0.06 0.2
      }}
    }}
  ]
  name "wall_s"
  boundingObject Box {{
    size {size_x:.6f} 0.06 0.2
  }}
}}
DEF WALL_E Solid {{
  translation {x_max:.6f} {center_y:.6f} 0.1
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.55 0.55 0.6
      }}
      geometry Box {{
        size 0.06 {size_y:.6f} 0.2
      }}
    }}
  ]
  name "wall_e"
  boundingObject Box {{
    size 0.06 {size_y:.6f} 0.2
  }}
}}
DEF WALL_W Solid {{
  translation {x_min:.6f} {center_y:.6f} 0.1
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.55 0.55 0.6
      }}
      geometry Box {{
        size 0.06 {size_y:.6f} 0.2
      }}
    }}
  ]
  name "wall_w"
  boundingObject Box {{
    size 0.06 {size_y:.6f} 0.2
  }}
}}
DEF GOAL Pose {{
  translation {float(scenario.goal[0]):.6f} {float(scenario.goal[1]):.6f} 0.02
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0 0.8 0.2
        roughness 0.6
        metalness 0
      }}
      geometry Cylinder {{
        height 0.02
        radius 0.18
      }}
    }}
  ]
}}
{obstacles_text}
DEF MFI_ROBOT Robot {{
  translation {float(scenario.start[0]):.6f} {float(scenario.start[1]):.6f} 0.035
  rotation 0 0 1 {robot_heading:.12f}
  children [
    Pose {{
      translation 0 0 0.025
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.15 0.15 0.18
            roughness 0.7
          }}
          geometry Cylinder {{
            height 0.05
            radius 0.05
          }}
        }}
      ]
    }}
    Pose {{
      translation 0 0.055 0.02
      rotation 1 0 0 1.57
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.05 0.05 0.05
          }}
          geometry Cylinder {{
            height 0.01
            radius 0.02
          }}
        }}
      ]
    }}
    Pose {{
      translation 0 -0.055 0.02
      rotation 1 0 0 1.57
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.05 0.05 0.05
          }}
          geometry Cylinder {{
            height 0.01
            radius 0.02
          }}
        }}
      ]
    }}
  ]
  name "mfi_robot"
  controller "mfi_epuck_python"
  supervisor TRUE
}}
"""


def _load_history(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return [{key: float(value) for key, value in row.items()} for row in rows]


def _plot_scenario(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]]) -> None:
    for method_name, history in histories.items():
        xs = [row["x"] for row in history]
        ys = [row["y"] for row in history]
        color = METHOD_SPECS[method_name]["color"]
        label = METHOD_SPECS[method_name]["label"]
        ax.plot(xs, ys, linewidth=2.0, label=label, color=color)
        ax.scatter(xs[-1], ys[-1], color=color, s=55, marker="x")
    for idx, obstacle in enumerate(scenario.obstacles.obstacles):
        patch = PolygonPatch(obstacle.vertices, closed=True, color="#d62728", alpha=0.18)
        ax.add_patch(patch)
        if idx == 0:
            centroid = np.mean(obstacle.vertices, axis=0)
            ax.annotate("obstacles", (float(centroid[0]), float(centroid[1])), textcoords="offset points", xytext=(6, 6))
    ax.scatter(scenario.start[0], scenario.start[1], color="#2ca02c", s=70, marker="o", label="start")
    ax.scatter(scenario.goal[0], scenario.goal[1], color="#111111", s=90, marker="*", label="goal")
    ax.set_title(f"{scenario.name.replace('_', ' ')} (webots e-puck)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)


def _run_one_scenario(scenario, method_name: str) -> tuple[list[dict[str, float]], dict[str, object], Path]:
    _ensure_generated_project_links()
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
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
    env["MFINAV_WEBOTS_MAX_STEPS"] = env.get("MFINAV_WEBOTS_MAX_STEPS", "2500")
    env["MFINAV_WEBOTS_QUIT"] = "1"
    env[METHOD_ENV] = method_name
    env[GOAL_TOLERANCE_ENV] = env.get(GOAL_TOLERANCE_ENV, "0.3")
    env[CONFIG_JSON_ENV] = json.dumps(METHOD_CONFIG_OVERRIDES.get(method_name, {}), separators=(",", ":"))
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
        raise RuntimeError(f"Webots did not produce a history for scenario {scenario.name} ({method_name}). See {console_path}.")
    history = _load_history(history_path)
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {"status": "unknown"}
    summary["webots_exit_code"] = result.returncode
    summary_path.write_text(json.dumps(summary, indent=2))
    return history, summary, world_path


def main() -> None:
    scenarios = [_rescale_scenario_for_webots(scenario) for scenario in make_default_scenarios()]
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    summary_rows: list[dict[str, str | float]] = []
    cols = min(3, len(scenarios))
    rows = math.ceil(len(scenarios) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 5.2 * rows))
    axes = np.atleast_1d(axes).ravel()

    for ax, scenario in zip(axes, scenarios):
        histories: dict[str, list[dict[str, float]]] = {}
        for method_name in METHOD_SPECS:
            history, summary, world_path = _run_one_scenario(scenario, method_name)
            histories[method_name] = history
            metrics = summary.get("metrics", {})
            summary_rows.append(
                {
                    "scenario": scenario.name,
                    "method": method_name,
                    "status": str(summary.get("status", "unknown")),
                    "success": float(metrics.get("success", 0.0)),
                    "goal_reached_once": float(metrics.get("goal_reached_once", 0.0)),
                    "steps": float(metrics.get("steps", len(history))),
                    "path_length": float(metrics.get("path_length", float("nan"))),
                    "final_goal_distance": float(metrics.get("final_goal_distance", float("nan"))),
                    "min_clearance": float(metrics.get("min_clearance", float("nan"))),
                    "mean_speed": float(metrics.get("mean_speed", float("nan"))),
                    "collision": float(metrics.get("collision", 0.0)),
                    "safety_violation": float(metrics.get("safety_violation", 0.0)),
                    "time_to_goal_steps": float(metrics.get("time_to_goal_steps", float("inf"))),
                    "path_efficiency": float(metrics.get("path_efficiency", float("nan"))),
                    "webots_exit_code": float(summary.get("webots_exit_code", -1)),
                    "world_path": str(world_path),
                }
            )
        _plot_scenario(ax, scenario, histories)

    for ax in axes[len(scenarios):]:
        ax.axis("off")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(PLOT_PNG, dpi=180)
    plt.close(fig)

    with METRICS_CSV.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "scenario",
                "method",
                "status",
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
                "webots_exit_code",
                "world_path",
            ],
        )
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

    print(f"benchmark_plot_webots_epuck={PLOT_PNG}")
    print(f"benchmark_metrics_webots_epuck={METRICS_CSV}")


if __name__ == "__main__":
    main()
