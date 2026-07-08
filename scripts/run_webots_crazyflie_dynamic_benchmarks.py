#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
from dataclasses import replace
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

from mfinav import DynamicObstacleCollection, MovingPrismObstacle, MovingSphereObstacle, PrismObstacle, SphereObstacle, make_dynamic_scenarios_3d  # noqa: E402
from mfinav.utils.paths import benchmark_artifact_dir, generated_world_dir  # noqa: E402


ARTIFACTS = benchmark_artifact_dir(ROOT, "webots_crazyflie_dynamic")
WORLD_DIR = generated_world_dir(ROOT, "crazyflie", "dynamic")
METRICS_CSV = ARTIFACTS / "benchmark_metrics_webots_crazyflie_dynamic.csv"
PLOT_PNG = ARTIFACTS / "benchmark_comparison_webots_crazyflie_dynamic.png"
SCENARIO_JSON_ENV = "MFINAV_WEBOTS_SCENARIO_JSON"
METHOD_ENV = "MFINAV_WEBOTS_METHOD"
CONFIG_JSON_ENV = "MFINAV_WEBOTS_CONFIG_JSON"
WEBOTS_TARGET_SPAN = 3.2
WEBOTS_MIN_Z = 0.22
WEBOTS_VIEW_MARGIN_Z = 0.75
METHOD_SPECS = {
    "paper_pd_3d": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric_3d": {"label": "MFI-Geometric", "color": "#2ca02c"},
}
METHOD_BASE_CONFIGS = {
    "paper_pd_3d": {
        "c_field": 10.0,
        "c_perp": 14.0,
        "speed_limit": 0.28,
        "kp_goal": 0.06,
        "kp_goal_relaxed": 0.06,
        "kd_goal": 0.65,
        "max_acceleration": 1.5,
        "max_speed_norm": 0.38,
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
    if not controllers_link.exists() and not controllers_link.is_symlink():
        controllers_link.symlink_to(target)


def _load_history(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return [{key: float(value) for key, value in row.items()} for row in rows]


def _scenario_extents_3d(scenario) -> tuple[float, float, float, float, float, float]:
    xs = [float(scenario.start[0]), float(scenario.goal[0])]
    ys = [float(scenario.start[1]), float(scenario.goal[1])]
    zs = [float(scenario.start[2]), float(scenario.goal[2])]
    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, (SphereObstacle, MovingSphereObstacle)):
            center = obstacle.center if hasattr(obstacle, "center") else obstacle.initial_center
            radius = float(obstacle.radius)
            xs.extend([float(center[0] - radius), float(center[0] + radius)])
            ys.extend([float(center[1] - radius), float(center[1] + radius)])
            zs.extend([float(center[2] - radius), float(center[2] + radius)])
        elif isinstance(obstacle, (PrismObstacle, MovingPrismObstacle)):
            vertices_xy = obstacle.vertices_xy if hasattr(obstacle, "vertices_xy") else obstacle.initial_vertices_xy
            z_min = obstacle.current_z_min if hasattr(obstacle, "current_z_min") else obstacle.z_min
            z_max = obstacle.current_z_max if hasattr(obstacle, "current_z_max") else obstacle.z_max
            xs.extend(float(vertex[0]) for vertex in vertices_xy)
            ys.extend(float(vertex[1]) for vertex in vertices_xy)
            zs.extend([float(z_min), float(z_max)])
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)


def _transform_scenario_for_webots(scenario):
    x_min, x_max, y_min, y_max, z_min, z_max = _scenario_extents_3d(scenario)
    dominant_span = max(x_max - x_min, y_max - y_min, z_max - z_min, 1e-9)
    scale = min(1.0, WEBOTS_TARGET_SPAN / dominant_span)
    z_shift = WEBOTS_MIN_Z - z_min * scale

    transformed_obstacles = []
    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, MovingSphereObstacle):
            transformed_obstacles.append(
                MovingSphereObstacle(
                    initial_center=obstacle.initial_center * scale + np.array([0.0, 0.0, z_shift], dtype=float),
                    radius=obstacle.radius * scale,
                    velocity=np.asarray(obstacle.velocity, dtype=float) * scale,
                    oscillation_amplitude=None if obstacle.oscillation_amplitude is None else np.asarray(obstacle.oscillation_amplitude, dtype=float) * scale,
                    oscillation_frequency=None if obstacle.oscillation_frequency is None else np.asarray(obstacle.oscillation_frequency, dtype=float),
                    oscillation_phase=None if obstacle.oscillation_phase is None else np.asarray(obstacle.oscillation_phase, dtype=float),
                )
            )
        elif isinstance(obstacle, SphereObstacle):
            transformed_obstacles.append(
                SphereObstacle(center=obstacle.center * scale + np.array([0.0, 0.0, z_shift], dtype=float), radius=obstacle.radius * scale)
            )
        elif isinstance(obstacle, MovingPrismObstacle):
            transformed_obstacles.append(
                MovingPrismObstacle(
                    initial_vertices_xy=obstacle.initial_vertices_xy * scale,
                    z_min=obstacle.z_min * scale + z_shift,
                    z_max=obstacle.z_max * scale + z_shift,
                    velocity=np.asarray(obstacle.velocity, dtype=float) * scale,
                    oscillation_amplitude=None if obstacle.oscillation_amplitude is None else np.asarray(obstacle.oscillation_amplitude, dtype=float) * scale,
                    oscillation_frequency=None if obstacle.oscillation_frequency is None else np.asarray(obstacle.oscillation_frequency, dtype=float),
                    oscillation_phase=None if obstacle.oscillation_phase is None else np.asarray(obstacle.oscillation_phase, dtype=float),
                )
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
            raise ValueError(f"Unsupported obstacle type in dynamic Crazyflie benchmark transform: {type(obstacle)!r}")

    return replace(
        scenario,
        start=scenario.start * scale + np.array([0.0, 0.0, z_shift], dtype=float),
        goal=scenario.goal * scale + np.array([0.0, 0.0, z_shift], dtype=float),
        obstacles=DynamicObstacleCollection(obstacles=transformed_obstacles),
        description=f"{scenario.description} Uniformly scaled by {scale:.3f} and lifted by {z_shift:.3f} m for Webots Crazyflie.",
    ), scale


def _scenario_to_payload(scenario) -> dict[str, object]:
    obstacles: list[dict[str, object]] = []
    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, MovingSphereObstacle):
            payload = obstacle.snapshot()
            payload["velocity"] = [float(x) for x in obstacle.velocity]
            if obstacle.oscillation_amplitude is not None:
                payload["oscillation_amplitude"] = [float(x) for x in obstacle.oscillation_amplitude]
            if obstacle.oscillation_frequency is not None:
                payload["oscillation_frequency"] = [float(x) for x in obstacle.oscillation_frequency]
            if obstacle.oscillation_phase is not None:
                payload["oscillation_phase"] = [float(x) for x in obstacle.oscillation_phase]
            obstacles.append(payload)
        elif isinstance(obstacle, SphereObstacle):
            obstacles.append(obstacle.snapshot())
        elif isinstance(obstacle, MovingPrismObstacle):
            payload = obstacle.snapshot()
            payload["velocity"] = [float(x) for x in obstacle.velocity]
            if obstacle.oscillation_amplitude is not None:
                payload["oscillation_amplitude"] = [float(x) for x in obstacle.oscillation_amplitude]
            if obstacle.oscillation_frequency is not None:
                payload["oscillation_frequency"] = [float(x) for x in obstacle.oscillation_frequency]
            if obstacle.oscillation_phase is not None:
                payload["oscillation_phase"] = [float(x) for x in obstacle.oscillation_phase]
            obstacles.append(payload)
        elif isinstance(obstacle, PrismObstacle):
            obstacles.append(obstacle.snapshot())
    return {
        "name": scenario.name,
        "description": scenario.description,
        "start": [float(scenario.start[0]), float(scenario.start[1]), float(scenario.start[2])],
        "goal": [float(scenario.goal[0]), float(scenario.goal[1]), float(scenario.goal[2])],
        "hover_height": float(max(scenario.start[2], 0.5)),
        "obstacles": obstacles,
    }


def _build_world_text(scenario) -> str:
    x_min, x_max, y_min, y_max, z_min, z_max = _scenario_extents_3d(scenario)
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
        obstacle.set_time(0.0)
        snapshot = obstacle.snapshot()
        if snapshot["kind"] == "sphere":
            center = snapshot["center"]
            radius = snapshot["radius"]
            obstacle_blocks.append(
                f"""DEF OBSTACLE_{index} Solid {{
  translation {center[0]:.6f} {center[1]:.6f} {center[2]:.6f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.85 0.25 0.25
        roughness 0.5
      }}
      geometry Sphere {{
        radius {radius:.6f}
      }}
    }}
  ]
  name "obstacle_{index}"
  boundingObject Sphere {{
    radius {radius:.6f}
  }}
}}"""
            )
        elif snapshot["kind"] == "prism":
            vertices = np.asarray(snapshot["vertices_xy"], dtype=float)
            x0, x1 = float(vertices[:, 0].min()), float(vertices[:, 0].max())
            y0, y1 = float(vertices[:, 1].min()), float(vertices[:, 1].max())
            z0, z1 = float(snapshot["z_min"]), float(snapshot["z_max"])
            obstacle_blocks.append(
                f"""DEF OBSTACLE_{index} Solid {{
  translation {(x0 + x1) * 0.5:.6f} {(y0 + y1) * 0.5:.6f} {(z0 + z1) * 0.5:.6f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.85 0.25 0.25
        roughness 0.5
      }}
      geometry Box {{
        size {x1 - x0:.6f} {y1 - y0:.6f} {z1 - z0:.6f}
      }}
    }}
  ]
  name "obstacle_{index}"
  boundingObject Box {{
    size {x1 - x0:.6f} {y1 - y0:.6f} {z1 - z0:.6f}
  }}
}}"""
            )

    obstacles_text = "\n".join(obstacle_blocks)
    floor_size = max(span_x, span_y, 3.5) + 1.0
    return f"""#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/bitcraze/crazyflie/protos/Crazyflie.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/Floor.proto"

WorldInfo {{
  title "MFI Webots Crazyflie dynamic benchmark - {scenario.name}"
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
        baseColor 0.0 0.8 0.2
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
}}
"""


def _method_overrides(scale: float, method_name: str) -> dict[str, float]:
    base = dict(METHOD_BASE_CONFIGS[method_name])
    base["r_l"] = 4.0 * scale
    base["r_la"] = 2.0 * scale
    base["sensor_range"] = 6.0 * scale
    return base


def _run_method(scenario, scale: float, method_name: str, scenario_dir: Path) -> dict[str, object]:
    history_path = scenario_dir / f"history_{method_name}.csv"
    summary_path = scenario_dir / f"summary_{method_name}.json"
    console_path = scenario_dir / f"console_{method_name}.log"
    world_path = scenario_dir / f"{scenario.name}.wbt"
    world_path.write_text(_build_world_text(scenario))

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
        raise RuntimeError(f"Webots Crazyflie dynamic benchmark did not produce a history for {scenario.name} ({method_name}). See {console_path}.")
    history = _load_history(history_path)
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {"status": "unknown"}
    summary["webots_exit_code"] = result.returncode
    summary_path.write_text(json.dumps(summary, indent=2))
    return {"history": history, "summary": summary, "world_path": world_path}


def _plot_scenario(ax_row: list[plt.Axes], scenario, histories: dict[str, list[dict[str, float]]]) -> None:
    projections = [("x", "y"), ("x", "z"), ("y", "z")]
    index_map = {"x": 0, "y": 1, "z": 2}

    for ax, (a, b) in zip(ax_row, projections):
        ia = index_map[a]
        ib = index_map[b]
        for method_name, history in histories.items():
            ax.plot(
                [row[a] for row in history],
                [row[b] for row in history],
                color=METHOD_SPECS[method_name]["color"],
                linewidth=2.0,
                label=METHOD_SPECS[method_name]["label"],
            )
            ax.scatter(history[-1][a], history[-1][b], color=METHOD_SPECS[method_name]["color"], marker="x", s=45)

        start = scenario.start
        goal = scenario.goal
        ax.scatter([start[ia]], [start[ib]], color="#16a34a", s=50, marker="o", label="start")
        ax.scatter([goal[ia]], [goal[ib]], color="#111111", s=85, marker="*", label="goal")

        for obstacle in scenario.obstacles.obstacles:
            obstacle.set_time(0.0)
            start_snapshot = obstacle.snapshot()
            obstacle.set_time(8.0)
            end_snapshot = obstacle.snapshot()
            obstacle.set_time(0.0)

            if start_snapshot["kind"] == "sphere":
                if (a, b) in {("x", "y"), ("x", "z"), ("y", "z")}:
                    start_c = start_snapshot["center"]
                    end_c = end_snapshot["center"]
                    circle = plt.Circle((start_c[ia], start_c[ib]), start_snapshot["radius"], color="#ef4444", alpha=0.12)
                    ax.add_patch(circle)
                    circle_end = plt.Circle((end_c[ia], end_c[ib]), end_snapshot["radius"], color="#f59e0b", alpha=0.10)
                    ax.add_patch(circle_end)
            elif start_snapshot["kind"] == "prism":
                start_vertices = np.asarray(start_snapshot["vertices_xy"], dtype=float)
                end_vertices = np.asarray(end_snapshot["vertices_xy"], dtype=float)
                if (a, b) == ("x", "y"):
                    ax.fill(start_vertices[:, 0], start_vertices[:, 1], color="#ef4444", alpha=0.12)
                    ax.fill(end_vertices[:, 0], end_vertices[:, 1], color="#f59e0b", alpha=0.10)
                elif (a, b) == ("x", "z"):
                    xs = start_vertices[:, 0]
                    ax.fill([xs.min(), xs.max(), xs.max(), xs.min()], [start_snapshot["z_min"], start_snapshot["z_min"], start_snapshot["z_max"], start_snapshot["z_max"]], color="#ef4444", alpha=0.12)
                    xs = end_vertices[:, 0]
                    ax.fill([xs.min(), xs.max(), xs.max(), xs.min()], [end_snapshot["z_min"], end_snapshot["z_min"], end_snapshot["z_max"], end_snapshot["z_max"]], color="#f59e0b", alpha=0.10)
                else:
                    ys = start_vertices[:, 1]
                    ax.fill([ys.min(), ys.max(), ys.max(), ys.min()], [start_snapshot["z_min"], start_snapshot["z_min"], start_snapshot["z_max"], start_snapshot["z_max"]], color="#ef4444", alpha=0.12)
                    ys = end_vertices[:, 1]
                    ax.fill([ys.min(), ys.max(), ys.max(), ys.min()], [end_snapshot["z_min"], end_snapshot["z_min"], end_snapshot["z_max"], end_snapshot["z_max"]], color="#f59e0b", alpha=0.10)

        ax.set_title(f"{scenario.name.replace('_', ' ')}: {a}-{b}")
        ax.set_xlabel(a)
        ax.set_ylabel(b)
        ax.grid(True, alpha=0.25)
        if (a, b) == ("x", "y"):
            ax.axis("equal")


def main() -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    _ensure_generated_project_links()

    source_scenarios = make_dynamic_scenarios_3d()[:3]
    transformed_scenarios = [_transform_scenario_for_webots(scenario) for scenario in source_scenarios]
    summary_rows: list[dict[str, str | float]] = []

    fig, axes = plt.subplots(len(transformed_scenarios), 3, figsize=(16, 4.8 * len(transformed_scenarios)))
    axes = np.atleast_2d(axes)

    for ax_row, (scenario, scale) in zip(axes, transformed_scenarios):
        scenario_dir = WORLD_DIR / scenario.name
        scenario_dir.mkdir(parents=True, exist_ok=True)
        histories: dict[str, list[dict[str, float]]] = {}
        for method_name in METHOD_SPECS:
            result = _run_method(scenario, scale, method_name, scenario_dir)
            histories[method_name] = result["history"]
            metrics = result["summary"].get("metrics", {})
            summary_rows.append(
                {
                    "scenario": scenario.name,
                    "method": method_name,
                    "status": str(result["summary"].get("status", "unknown")),
                    "success": float(metrics.get("success", 0.0)),
                    "goal_reached_once": float(metrics.get("goal_reached_once", 0.0)),
                    "steps": float(metrics.get("steps", 0.0)),
                    "path_length": float(metrics.get("path_length", 0.0)),
                    "final_goal_distance": float(metrics.get("final_goal_distance", 0.0)),
                    "min_clearance": float(metrics.get("min_clearance", float("inf"))),
                    "mean_speed": float(metrics.get("mean_speed", 0.0)),
                    "collision": float(metrics.get("collision", 0.0)),
                    "safety_violation": float(metrics.get("safety_violation", 0.0)),
                    "time_to_goal_steps": float(metrics.get("time_to_goal_steps", float("inf"))),
                    "path_efficiency": float(metrics.get("path_efficiency", 0.0)),
                    "webots_exit_code": float(result["summary"].get("webots_exit_code", -1)),
                    "world_path": str(result["world_path"]),
                    "scale": float(scale),
                }
            )
        _plot_scenario(list(ax_row), scenario, histories)

    handles, labels = axes[0][0].get_legend_handles_labels()
    dedup: dict[str, object] = {}
    for handle, label in zip(handles, labels):
        dedup.setdefault(label, handle)
    fig.legend(dedup.values(), dedup.keys(), loc="upper center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(PLOT_PNG, dpi=180)
    plt.close(fig)

    with METRICS_CSV.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)

    for row in summary_rows:
        print(
            f"{row['scenario']} {row['method']}: "
            f"status={row['status']} success={int(row['success'])} "
            f"collision={int(row['collision'])} final_goal_distance={row['final_goal_distance']:.3f} "
            f"min_clearance={row['min_clearance']:.3f}"
        )

    print(f"benchmark_plot_webots_crazyflie_dynamic={PLOT_PNG}")
    print(f"benchmark_metrics_webots_crazyflie_dynamic={METRICS_CSV}")


if __name__ == "__main__":
    main()
