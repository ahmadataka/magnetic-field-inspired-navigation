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

from mfinav import CircleObstacle, MovingCircleObstacle, MovingPolygonObstacle, PolygonObstacle, make_dynamic_scenarios_2d  # noqa: E402
from mfinav.utils.paths import benchmark_artifact_dir, generated_world_dir  # noqa: E402
from run_benchmarks_dynamic_2d import METHOD_SPECS as BASE_METHOD_SPECS, _make_metrics_row, _plot_static_snapshot  # noqa: E402


ARTIFACTS = benchmark_artifact_dir(ROOT, "webots_epuck_dynamic")
WORLD_DIR = generated_world_dir(ROOT, "epuck", "dynamic")
METRICS_CSV = ARTIFACTS / "benchmark_metrics_dynamic_webots_epuck.csv"
PLOT_PNG = ARTIFACTS / "benchmark_comparison_dynamic_webots_epuck.png"
SCENARIO_JSON_ENV = "MFINAV_WEBOTS_SCENARIO_JSON"
METHOD_ENV = "MFINAV_WEBOTS_METHOD"
GOAL_TOLERANCE_ENV = "MFINAV_WEBOTS_GOAL_TOLERANCE"
CONFIG_JSON_ENV = "MFINAV_WEBOTS_CONFIG_JSON"
WEBOTS_TARGET_MAX_SPAN = 4.5
WEBOTS_MARGIN = 1.0
WEBOTS_HORIZON_S = 160.0
METHOD_SPECS = {
    "paper_pd": BASE_METHOD_SPECS["paper_pd"],
    "paper_geometric": BASE_METHOD_SPECS["paper_geometric"],
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
}


def _ensure_generated_project_links() -> None:
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    controllers_link = WORLD_DIR / "controllers"
    target = ROOT / "webots" / "controllers"
    if controllers_link.exists() or controllers_link.is_symlink():
        return
    controllers_link.symlink_to(target)


def _scale_obstacle(obstacle, scale: float):
    if isinstance(obstacle, CircleObstacle):
        return CircleObstacle(center=np.asarray(obstacle.center, dtype=float) * scale, radius=float(obstacle.radius) * scale)
    if isinstance(obstacle, PolygonObstacle):
        return PolygonObstacle(vertices=np.asarray(obstacle.vertices, dtype=float) * scale)
    if isinstance(obstacle, MovingCircleObstacle):
        return MovingCircleObstacle(
            initial_center=np.asarray(obstacle.initial_center, dtype=float) * scale,
            radius=float(obstacle.radius) * scale,
            velocity=np.asarray(obstacle.velocity, dtype=float) * scale,
            oscillation_amplitude=None if obstacle.oscillation_amplitude is None else np.asarray(obstacle.oscillation_amplitude, dtype=float) * scale,
            oscillation_frequency=None if obstacle.oscillation_frequency is None else np.asarray(obstacle.oscillation_frequency, dtype=float),
            oscillation_phase=None if obstacle.oscillation_phase is None else np.asarray(obstacle.oscillation_phase, dtype=float),
        )
    if isinstance(obstacle, MovingPolygonObstacle):
        return MovingPolygonObstacle(
            initial_vertices=np.asarray(obstacle.initial_vertices, dtype=float) * scale,
            velocity=np.asarray(obstacle.velocity, dtype=float) * scale,
            oscillation_amplitude=None if obstacle.oscillation_amplitude is None else np.asarray(obstacle.oscillation_amplitude, dtype=float) * scale,
            oscillation_frequency=None if obstacle.oscillation_frequency is None else np.asarray(obstacle.oscillation_frequency, dtype=float),
            oscillation_phase=None if obstacle.oscillation_phase is None else np.asarray(obstacle.oscillation_phase, dtype=float),
        )
    raise ValueError(f"Unsupported dynamic obstacle type for Webots scaling: {type(obstacle)!r}")


def _rescale_scenario_for_webots(scenario):
    xs = [float(scenario.start[0]), float(scenario.goal[0])]
    ys = [float(scenario.start[1]), float(scenario.goal[1])]
    sample_times = np.linspace(0.0, WEBOTS_HORIZON_S, 8).tolist()
    snapshots = scenario.obstacles.snapshots_over_time(sample_times)
    for snapshot in snapshots:
        for obstacle in snapshot["obstacles"]:
            if obstacle["kind"] == "circle":
                center = obstacle["center"]
                radius = float(obstacle["radius"])
                xs.extend([float(center[0] - radius), float(center[0] + radius)])
                ys.extend([float(center[1] - radius), float(center[1] + radius)])
            elif obstacle["kind"] == "polygon":
                xs.extend(float(vertex[0]) for vertex in obstacle["vertices"])
                ys.extend(float(vertex[1]) for vertex in obstacle["vertices"])
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    dominant_span = max(span_x, span_y, 1e-9)
    scale = min(1.0, WEBOTS_TARGET_MAX_SPAN / dominant_span)
    if abs(scale - 1.0) < 1e-9:
        return scenario
    scaled_obstacles = [_scale_obstacle(obstacle, scale) for obstacle in scenario.obstacles.obstacles]
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
            raise ValueError("Failed to triangulate polygon for dynamic Webots world generation.")
    if len(remaining) == 3:
        triangles.append((remaining[0], remaining[1], remaining[2]))
    return triangles


def _local_polygon_shape_block(local_vertices: np.ndarray, height: float = 0.12) -> str:
    triangles = _triangulate_polygon(local_vertices)
    points: list[str] = []
    n = len(local_vertices)
    for x, y in local_vertices:
        points.append(f"{float(x):.6f} {float(y):.6f} 0")
    for x, y in local_vertices:
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


def _circle_shape_block(radius: float, height: float = 0.12) -> str:
    return f"""Shape {{
          appearance PBRAppearance {{
            baseColor 0.75 0.2 0.2
          }}
          geometry Cylinder {{
            height {height:.6f}
            radius {radius:.6f}
          }}
        }}"""


def _obstacle_spec_and_block(obstacle, index: int) -> tuple[dict[str, object], str]:
    obstacle_def = f"OBSTACLE_{index + 1}"
    if isinstance(obstacle, MovingCircleObstacle):
        center = np.asarray(obstacle.initial_center, dtype=float)
        block = f"""DEF {obstacle_def} Solid {{
  translation {float(center[0]):.6f} {float(center[1]):.6f} 0.06
  children [
    {_circle_shape_block(float(obstacle.radius))}
  ]
  name "obstacle_{index + 1}"
  boundingObject Cylinder {{
    height 0.12
    radius {float(obstacle.radius):.6f}
  }}
}}"""
        spec = {
            "def": obstacle_def,
            "kind": "circle",
            "center": [float(center[0]), float(center[1])],
            "radius": float(obstacle.radius),
            "velocity": [float(obstacle.velocity[0]), float(obstacle.velocity[1])],
        }
        if obstacle.oscillation_amplitude is not None:
            spec["oscillation_amplitude"] = [float(value) for value in obstacle.oscillation_amplitude]
        if obstacle.oscillation_frequency is not None:
            spec["oscillation_frequency"] = [float(value) for value in obstacle.oscillation_frequency]
        if obstacle.oscillation_phase is not None:
            spec["oscillation_phase"] = [float(value) for value in obstacle.oscillation_phase]
        return spec, block

    if isinstance(obstacle, CircleObstacle):
        center = np.asarray(obstacle.center, dtype=float)
        block = f"""DEF {obstacle_def} Solid {{
  translation {float(center[0]):.6f} {float(center[1]):.6f} 0.06
  children [
    {_circle_shape_block(float(obstacle.radius))}
  ]
  name "obstacle_{index + 1}"
  boundingObject Cylinder {{
    height 0.12
    radius {float(obstacle.radius):.6f}
  }}
}}"""
        spec = {
            "def": obstacle_def,
            "kind": "circle",
            "center": [float(center[0]), float(center[1])],
            "radius": float(obstacle.radius),
        }
        return spec, block

    if isinstance(obstacle, MovingPolygonObstacle):
        vertices = np.asarray(obstacle.initial_vertices, dtype=float)
        centroid = np.mean(vertices, axis=0)
        local_vertices = vertices - centroid
        shape_block = _local_polygon_shape_block(local_vertices)
        block = f"""DEF {obstacle_def} Solid {{
  translation {float(centroid[0]):.6f} {float(centroid[1]):.6f} 0.0
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
        spec = {
            "def": obstacle_def,
            "kind": "polygon",
            "initial_vertices": [[float(vertex[0]), float(vertex[1])] for vertex in vertices],
            "velocity": [float(obstacle.velocity[0]), float(obstacle.velocity[1])],
        }
        if obstacle.oscillation_amplitude is not None:
            spec["oscillation_amplitude"] = [float(value) for value in obstacle.oscillation_amplitude]
        if obstacle.oscillation_frequency is not None:
            spec["oscillation_frequency"] = [float(value) for value in obstacle.oscillation_frequency]
        if obstacle.oscillation_phase is not None:
            spec["oscillation_phase"] = [float(value) for value in obstacle.oscillation_phase]
        return spec, block

    if isinstance(obstacle, PolygonObstacle):
        vertices = np.asarray(obstacle.vertices, dtype=float)
        centroid = np.mean(vertices, axis=0)
        local_vertices = vertices - centroid
        shape_block = _local_polygon_shape_block(local_vertices)
        block = f"""DEF {obstacle_def} Solid {{
  translation {float(centroid[0]):.6f} {float(centroid[1]):.6f} 0.0
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
        spec = {
            "def": obstacle_def,
            "kind": "polygon",
            "vertices": [[float(vertex[0]), float(vertex[1])] for vertex in vertices],
        }
        return spec, block

    raise ValueError(f"Unsupported dynamic Webots obstacle type: {type(obstacle)!r}")


def _scenario_to_payload(scenario) -> dict[str, object]:
    heading = math.atan2(float(scenario.goal[1] - scenario.start[1]), float(scenario.goal[0] - scenario.start[0]))
    obstacles = []
    for index, obstacle in enumerate(scenario.obstacles.obstacles):
        spec, _ = _obstacle_spec_and_block(obstacle, index)
        obstacles.append(spec)
    return {
        "name": scenario.name,
        "description": scenario.description,
        "start": [float(scenario.start[0]), float(scenario.start[1])],
        "goal": [float(scenario.goal[0]), float(scenario.goal[1])],
        "heading": heading,
        "obstacles": obstacles,
    }


def _bounds_for_scenario(scenario) -> tuple[float, float, float, float]:
    xs = [float(scenario.start[0]), float(scenario.goal[0])]
    ys = [float(scenario.start[1]), float(scenario.goal[1])]
    snapshots = scenario.obstacles.snapshots_over_time(np.linspace(0.0, WEBOTS_HORIZON_S, 8).tolist())
    for snapshot in snapshots:
        for obstacle in snapshot["obstacles"]:
            if obstacle["kind"] == "circle":
                center = obstacle["center"]
                radius = float(obstacle["radius"])
                xs.extend([float(center[0] - radius), float(center[0] + radius)])
                ys.extend([float(center[1] - radius), float(center[1] + radius)])
            elif obstacle["kind"] == "polygon":
                xs.extend(float(vertex[0]) for vertex in obstacle["vertices"])
                ys.extend(float(vertex[1]) for vertex in obstacle["vertices"])
    return min(xs) - WEBOTS_MARGIN, max(xs) + WEBOTS_MARGIN, min(ys) - WEBOTS_MARGIN, max(ys) + WEBOTS_MARGIN


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
        _, block = _obstacle_spec_and_block(obstacle, index)
        obstacle_blocks.append(block)
    obstacles_text = "\n".join(obstacle_blocks)
    return f"""#VRML_SIM R2023b utf8

WorldInfo {{
  title "MFI Webots dynamic benchmark - {scenario.name}"
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


def _run_one_scenario(scenario, method_name: str) -> tuple[list[dict[str, float]], dict[str, object], Path]:
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
        raise RuntimeError(f"Webots did not produce a dynamic history for {scenario.name} ({method_name}). See {console_path}.")
    history = _load_history(history_path)
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {"status": "unknown"}
    summary["webots_exit_code"] = result.returncode
    summary_path.write_text(json.dumps(summary, indent=2))
    return history, summary, world_path


def main() -> None:
    scenarios = [_rescale_scenario_for_webots(scenario) for scenario in make_dynamic_scenarios_2d()]
    ARTIFACTS.mkdir(parents=True, exist_ok=True)

    summary_rows: list[dict[str, str | float]] = []
    cols = min(2, len(scenarios))
    rows = math.ceil(len(scenarios) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(7 * cols, 5.4 * rows))
    axes = np.atleast_1d(axes).ravel()

    for ax, scenario in zip(axes, scenarios):
        histories: dict[str, list[dict[str, float]]] = {}
        for method_name in METHOD_SPECS:
            history, summary, world_path = _run_one_scenario(scenario, method_name)
            histories[method_name] = history
            metrics = summary.get("metrics", {})
            row = _make_metrics_row(scenario.name, method_name, metrics)
            row["status"] = str(summary.get("status", "unknown"))
            row["webots_exit_code"] = float(summary.get("webots_exit_code", -1))
            row["world_path"] = str(world_path)
            summary_rows.append(row)

        sample_times_for_png = np.linspace(0.0, WEBOTS_HORIZON_S, 4).tolist()
        _plot_static_snapshot(ax, scenario, histories, sample_times_for_png)
        ax.set_title(f"{scenario.name.replace('_', ' ')} (webots e-puck)")

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
                "status",
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

    print(f"benchmark_plot_dynamic_webots_epuck={PLOT_PNG}")
    print(f"benchmark_metrics_dynamic_webots_epuck={METRICS_CSV}")


if __name__ == "__main__":
    main()
