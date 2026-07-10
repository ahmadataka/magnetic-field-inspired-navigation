#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle as CirclePatch
from matplotlib.patches import FancyArrowPatch
from matplotlib.patches import Polygon as PolygonPatch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
SCRIPTS = ROOT / "scripts"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from mfinav import (  # noqa: E402
    ArtificialPotentialFieldNavigator,
    DoubleIntegratorState,
    MagneticFieldNavigator3D,
    make_dynamic_scenarios_2d,
    make_dynamic_scenarios_3d,
    make_paper_geometric_3d_config,
    make_paper_pd_3d_config,
    simulate,
)
from mfinav.utils.paths import summary_artifact_dir  # noqa: E402
import run_webots_crazyflie_dynamic_benchmarks as crazyflie_dynamic  # noqa: E402
import run_webots_epuck_dynamic_benchmarks as epuck_dynamic  # noqa: E402


OUTPUT_DIR = summary_artifact_dir(ROOT, "dynamic_paper_figures")
DOC_PATH = ROOT / "docs" / "dynamic_paper_figures.md"

METHOD_SPECS_2D = {
    "paper_pd": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric": {"label": "MFI-Geometric", "color": "#2ca02c"},
    "apf": {"label": "APF", "color": "#ff7f0e"},
}

METHOD_SPECS_3D = {
    "paper_pd_3d": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric_3d": {"label": "MFI-Geometric", "color": "#2ca02c"},
    "apf_3d": {"label": "APF", "color": "#ff7f0e"},
}


def _initial_state(start: np.ndarray) -> DoubleIntegratorState:
    return DoubleIntegratorState(position=np.asarray(start, dtype=float).copy(), velocity=np.zeros_like(start, dtype=float))


def _lookup_scenario(scenarios, name: str):
    for scenario in scenarios:
        if scenario.name == name:
            return scenario
    raise KeyError(f"Scenario {name} not found.")


def _figure_legend(method_specs: dict[str, dict[str, str]]) -> list[Line2D]:
    handles = [
        Line2D([0], [0], color=spec["color"], linewidth=2.8, label=spec["label"])
        for spec in method_specs.values()
    ]
    handles.extend(
        [
            Line2D([0], [0], marker="o", color="w", markerfacecolor="#16a34a", markersize=10, label="start"),
            Line2D([0], [0], marker="*", color="w", markerfacecolor="#111111", markersize=14, label="goal"),
            Line2D([0], [0], color="#b91c1c", linewidth=8, alpha=0.18, label="obstacle snapshots"),
        ]
    )
    return handles


def _center_of_shape(shape: dict[str, object]) -> np.ndarray:
    if shape["kind"] in {"circle", "sphere"}:
        return np.asarray(shape["center"], dtype=float)
    if shape["kind"] == "polygon":
        return np.mean(np.asarray(shape["vertices"], dtype=float), axis=0)
    if shape["kind"] == "prism":
        vertices_xy = np.asarray(shape["vertices_xy"], dtype=float)
        center_xy = np.mean(vertices_xy, axis=0)
        center_z = 0.5 * (float(shape["z_min"]) + float(shape["z_max"]))
        return np.array([center_xy[0], center_xy[1], center_z], dtype=float)
    raise ValueError(f"Unsupported shape kind {shape['kind']}.")


def _sample_snapshot_times(history: list[dict[str, float]]) -> list[float]:
    if not history:
        return [0.0]
    final_time = float(history[-1].get("time", len(history) - 1))
    return [0.0, 0.45 * final_time, 0.85 * final_time]


def _set_axes_limits_2d(ax: plt.Axes, xs: list[float], ys: list[float]) -> None:
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_span = max(x_max - x_min, 1.0)
    y_span = max(y_max - y_min, 1.0)
    ax.set_xlim(x_min - 0.12 * x_span, x_max + 0.12 * x_span)
    ax.set_ylim(y_min - 0.14 * y_span, y_max + 0.14 * y_span)


def _plot_2d_shape(ax: plt.Axes, shape: dict[str, object], alpha: float) -> None:
    if shape["kind"] == "circle":
        ax.add_patch(
            CirclePatch(
                tuple(shape["center"]),
                float(shape["radius"]),
                facecolor="#dc2626",
                edgecolor="#991b1b",
                linewidth=1.2,
                alpha=alpha,
            )
        )
        return
    if shape["kind"] == "polygon":
        ax.add_patch(
            PolygonPatch(
                np.asarray(shape["vertices"], dtype=float),
                closed=True,
                facecolor="#dc2626",
                edgecolor="#991b1b",
                linewidth=1.2,
                alpha=alpha,
            )
        )
        return
    raise ValueError(f"Unsupported 2D shape kind {shape['kind']}.")


def _plot_snapshot_motion_arrow_2d(ax: plt.Axes, snapshots: list[list[dict[str, object]]]) -> None:
    if len(snapshots) < 2:
        return
    for obstacle_index in range(len(snapshots[0])):
        start_center = _center_of_shape(snapshots[0][obstacle_index])
        end_center = _center_of_shape(snapshots[-1][obstacle_index])
        ax.add_patch(
            FancyArrowPatch(
                (start_center[0], start_center[1]),
                (end_center[0], end_center[1]),
                arrowstyle="->",
                mutation_scale=13,
                linewidth=1.6,
                linestyle="--",
                color="#7f1d1d",
                alpha=0.8,
            )
        )


def _plot_publication_2d(ax: plt.Axes, scenario, histories: dict[str, list[dict[str, float]]], method_specs: dict[str, dict[str, str]]) -> None:
    all_x: list[float] = [float(scenario.start[0]), float(scenario.goal[0])]
    all_y: list[float] = [float(scenario.start[1]), float(scenario.goal[1])]
    for method_name, history in histories.items():
        xs = [row["x"] for row in history]
        ys = [row["y"] for row in history]
        all_x.extend(xs)
        all_y.extend(ys)
        ax.plot(xs, ys, color=method_specs[method_name]["color"], linewidth=2.8, label=method_specs[method_name]["label"])
        ax.scatter(xs[-1], ys[-1], color=method_specs[method_name]["color"], s=54, marker="x", zorder=5)

    sample_times = _sample_snapshot_times(next(iter(histories.values())))
    snapshots: list[list[dict[str, object]]] = []
    for time_s in sample_times:
        scenario.obstacles.set_time(time_s)
        snapshot = scenario.obstacles.snapshot()["obstacles"]
        snapshots.append(snapshot)
        for shape in snapshot:
            _plot_2d_shape(ax, shape, alpha=0.12 + 0.10 * len(snapshots))
            if shape["kind"] == "circle":
                center = np.asarray(shape["center"], dtype=float)
                radius = float(shape["radius"])
                all_x.extend([center[0] - radius, center[0] + radius])
                all_y.extend([center[1] - radius, center[1] + radius])
            elif shape["kind"] == "polygon":
                vertices = np.asarray(shape["vertices"], dtype=float)
                all_x.extend(vertices[:, 0].tolist())
                all_y.extend(vertices[:, 1].tolist())
    scenario.obstacles.set_time(0.0)
    _plot_snapshot_motion_arrow_2d(ax, snapshots)

    ax.scatter(scenario.start[0], scenario.start[1], color="#16a34a", s=90, marker="o", zorder=6)
    ax.scatter(scenario.goal[0], scenario.goal[1], color="#111111", s=140, marker="*", zorder=6)
    ax.grid(True, alpha=0.22)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    _set_axes_limits_2d(ax, all_x, all_y)


def _project_prism(shape: dict[str, object], axes: tuple[str, str]) -> tuple[list[float], list[float]]:
    if axes == ("x", "y"):
        vertices = np.asarray(shape["vertices_xy"], dtype=float)
        return vertices[:, 0].tolist(), vertices[:, 1].tolist()
    if axes == ("x", "z"):
        vertices = np.asarray(shape["vertices_xy"], dtype=float)
        x_min = float(np.min(vertices[:, 0]))
        x_max = float(np.max(vertices[:, 0]))
        return [x_min, x_max, x_max, x_min], [float(shape["z_min"]), float(shape["z_min"]), float(shape["z_max"]), float(shape["z_max"])]
    if axes == ("y", "z"):
        vertices = np.asarray(shape["vertices_xy"], dtype=float)
        y_min = float(np.min(vertices[:, 1]))
        y_max = float(np.max(vertices[:, 1]))
        return [y_min, y_max, y_max, y_min], [float(shape["z_min"]), float(shape["z_min"]), float(shape["z_max"]), float(shape["z_max"])]
    raise ValueError(f"Unsupported projection {axes}.")


def _plot_projected_shape(ax: plt.Axes, shape: dict[str, object], axes: tuple[str, str], alpha: float) -> None:
    axis_map = {"x": 0, "y": 1, "z": 2}
    if shape["kind"] == "sphere":
        center = np.asarray(shape["center"], dtype=float)
        ax.add_patch(
            CirclePatch(
                (center[axis_map[axes[0]]], center[axis_map[axes[1]]]),
                float(shape["radius"]),
                facecolor="#dc2626",
                edgecolor="#991b1b",
                linewidth=1.1,
                alpha=alpha,
            )
        )
        return
    if shape["kind"] == "prism":
        xs, ys = _project_prism(shape, axes)
        ax.add_patch(
            PolygonPatch(
                np.column_stack((xs, ys)),
                closed=True,
                facecolor="#dc2626",
                edgecolor="#991b1b",
                linewidth=1.1,
                alpha=alpha,
            )
        )
        return
    raise ValueError(f"Unsupported 3D shape kind {shape['kind']}.")


def _plot_snapshot_motion_arrow_projected(ax: plt.Axes, snapshots: list[list[dict[str, object]]], axes: tuple[str, str]) -> None:
    if len(snapshots) < 2:
        return
    axis_map = {"x": 0, "y": 1, "z": 2}
    for obstacle_index in range(len(snapshots[0])):
        start_center = _center_of_shape(snapshots[0][obstacle_index])
        end_center = _center_of_shape(snapshots[-1][obstacle_index])
        ax.add_patch(
            FancyArrowPatch(
                (start_center[axis_map[axes[0]]], start_center[axis_map[axes[1]]]),
                (end_center[axis_map[axes[0]]], end_center[axis_map[axes[1]]]),
                arrowstyle="->",
                mutation_scale=12,
                linewidth=1.5,
                linestyle="--",
                color="#7f1d1d",
                alpha=0.8,
            )
        )


def _set_axes_limits(ax: plt.Axes, coords_a: list[float], coords_b: list[float]) -> None:
    min_a, max_a = min(coords_a), max(coords_a)
    min_b, max_b = min(coords_b), max(coords_b)
    span_a = max(max_a - min_a, 1.0)
    span_b = max(max_b - min_b, 1.0)
    ax.set_xlim(min_a - 0.12 * span_a, max_a + 0.12 * span_a)
    ax.set_ylim(min_b - 0.14 * span_b, max_b + 0.14 * span_b)


def _plot_publication_3d(fig: plt.Figure, axes: list[plt.Axes], scenario, histories: dict[str, list[dict[str, float]]], method_specs: dict[str, dict[str, str]], projections: list[tuple[str, str]]) -> None:
    axis_map = {"x": 0, "y": 1, "z": 2}
    sample_times = _sample_snapshot_times(next(iter(histories.values())))
    snapshots: list[list[dict[str, object]]] = []
    for time_s in sample_times:
        scenario.obstacles.set_time(time_s)
        snapshots.append(scenario.obstacles.snapshot()["obstacles"])
    scenario.obstacles.set_time(0.0)

    for ax, projection in zip(axes, projections):
        a_key, b_key = projection
        coords_a = [float(scenario.start[axis_map[a_key]]), float(scenario.goal[axis_map[a_key]])]
        coords_b = [float(scenario.start[axis_map[b_key]]), float(scenario.goal[axis_map[b_key]])]
        for method_name, history in histories.items():
            xs = [row[a_key] for row in history]
            ys = [row[b_key] for row in history]
            coords_a.extend(xs)
            coords_b.extend(ys)
            ax.plot(xs, ys, color=method_specs[method_name]["color"], linewidth=2.7)
            ax.scatter(xs[-1], ys[-1], color=method_specs[method_name]["color"], s=52, marker="x", zorder=5)

        for snap_index, snapshot in enumerate(snapshots):
            for shape in snapshot:
                _plot_projected_shape(ax, shape, projection, alpha=0.12 + 0.09 * (snap_index + 1))
                if shape["kind"] == "sphere":
                    center = np.asarray(shape["center"], dtype=float)
                    radius = float(shape["radius"])
                    coords_a.extend([center[axis_map[a_key]] - radius, center[axis_map[a_key]] + radius])
                    coords_b.extend([center[axis_map[b_key]] - radius, center[axis_map[b_key]] + radius])
                elif shape["kind"] == "prism":
                    xs, ys = _project_prism(shape, projection)
                    coords_a.extend(xs)
                    coords_b.extend(ys)
        _plot_snapshot_motion_arrow_projected(ax, snapshots, projection)

        ax.scatter(scenario.start[axis_map[a_key]], scenario.start[axis_map[b_key]], color="#16a34a", s=88, marker="o", zorder=6)
        ax.scatter(scenario.goal[axis_map[a_key]], scenario.goal[axis_map[b_key]], color="#111111", s=138, marker="*", zorder=6)
        ax.set_xlabel(f"{a_key} [m]")
        ax.set_ylabel(f"{b_key} [m]")
        ax.grid(True, alpha=0.22)
        _set_axes_limits(ax, coords_a, coords_b)
        if projection == ("x", "y"):
            ax.set_aspect("equal", adjustable="box")

    fig.legend(
        handles=_figure_legend(method_specs),
        loc="upper center",
        ncol=6,
        frameon=False,
        bbox_to_anchor=(0.5, 1.02),
    )


def _load_cached_result(history_path: Path, summary_path: Path) -> tuple[list[dict[str, float]], dict[str, object]] | None:
    if not history_path.exists() or not summary_path.exists():
        return None
    return (
        epuck_dynamic._load_history(history_path),
        json.loads(summary_path.read_text()),
    )


def _run_or_load_epuck(scenario, method_name: str) -> tuple[list[dict[str, float]], dict[str, object], Path]:
    scenario_dir = epuck_dynamic.WORLD_DIR / scenario.name
    history_path = scenario_dir / f"history_{method_name}.csv"
    summary_path = scenario_dir / f"summary_{method_name}.json"
    world_path = scenario_dir / f"{scenario.name}.wbt"
    cached = _load_cached_result(history_path, summary_path)
    if cached is not None:
        history, summary = cached
        return history, summary, world_path
    return epuck_dynamic._run_one_scenario(scenario, method_name)


def _run_or_load_crazyflie(scenario, scale: float, method_name: str) -> tuple[list[dict[str, float]], dict[str, object], Path]:
    scenario_dir = crazyflie_dynamic.WORLD_DIR / scenario.name
    scenario_dir.mkdir(parents=True, exist_ok=True)
    history_path = scenario_dir / f"history_{method_name}.csv"
    summary_path = scenario_dir / f"summary_{method_name}.json"
    world_path = scenario_dir / f"{scenario.name}.wbt"
    cached = _load_cached_result(history_path, summary_path)
    if cached is not None:
        history, summary = cached
        return history, summary, world_path
    result = crazyflie_dynamic._run_method(scenario, scale, method_name, scenario_dir)
    return result["history"], result["summary"], result["world_path"]


def _save(fig: plt.Figure, stem: str) -> tuple[Path, Path]:
    png_path = OUTPUT_DIR / f"{stem}.png"
    pdf_path = OUTPUT_DIR / f"{stem}.pdf"
    fig.savefig(png_path, dpi=220, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)
    return png_path, pdf_path


def _make_double_integrator_3d_figure() -> dict[str, str]:
    scenarios = make_dynamic_scenarios_3d()
    scenario = _lookup_scenario(scenarios, "moving_prism_gate_3d")
    config_pd = make_paper_pd_3d_config()
    config_geometric = make_paper_geometric_3d_config()
    histories = {
        "paper_pd_3d": simulate(_initial_state(scenario.start), scenario.goal, scenario.obstacles, config_pd, navigator=MagneticFieldNavigator3D(config_pd)),
        "paper_geometric_3d": simulate(_initial_state(scenario.start), scenario.goal, scenario.obstacles, config_geometric, navigator=MagneticFieldNavigator3D(config_geometric)),
        "apf_3d": simulate(_initial_state(scenario.start), scenario.goal, scenario.obstacles, config_pd, navigator=ArtificialPotentialFieldNavigator(config_pd)),
    }
    fig, axes = plt.subplots(1, 2, figsize=(12.6, 5.1))
    _plot_publication_3d(fig, list(axes), scenario, histories, METHOD_SPECS_3D, [("x", "y"), ("x", "z")])
    fig.suptitle("Dynamic 3D Double-Integrator: Moving Prism Gate", y=1.08, fontsize=15)
    png_path, pdf_path = _save(fig, "figure_dynamic_double_integrator_3d")
    return {
        "title": "Dynamic 3D double-integrator",
        "scenario": scenario.name,
        "description": scenario.description,
        "png": str(png_path.relative_to(ROOT)),
        "pdf": str(pdf_path.relative_to(ROOT)),
    }


def _make_epuck_figure() -> dict[str, str]:
    scenarios = [_lookup_scenario(make_dynamic_scenarios_2d(), "moving_u_shape")]
    scenario = epuck_dynamic._rescale_scenario_for_webots(scenarios[0])
    histories: dict[str, list[dict[str, float]]] = {}
    for method_name in METHOD_SPECS_2D:
        history, _summary, _world = _run_or_load_epuck(scenario, method_name)
        histories[method_name] = history
    fig, ax = plt.subplots(figsize=(7.5, 5.8))
    _plot_publication_2d(ax, scenario, histories, METHOD_SPECS_2D)
    ax.legend(handles=_figure_legend(METHOD_SPECS_2D), loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.18))
    ax.set_title("Webots e-puck Dynamic Benchmark: Moving U-Shape", fontsize=14)
    fig.tight_layout()
    png_path, pdf_path = _save(fig, "figure_dynamic_webots_epuck")
    return {
        "title": "Webots e-puck dynamic benchmark",
        "scenario": scenario.name,
        "description": scenario.description,
        "png": str(png_path.relative_to(ROOT)),
        "pdf": str(pdf_path.relative_to(ROOT)),
    }


def _make_crazyflie_figure() -> dict[str, str]:
    source_scenario = _lookup_scenario(make_dynamic_scenarios_3d(), "moving_sphere_crossing_3d")
    scenario, scale = crazyflie_dynamic._transform_scenario_for_webots(source_scenario)
    histories: dict[str, list[dict[str, float]]] = {}
    for method_name in METHOD_SPECS_3D:
        history, _summary, _world = _run_or_load_crazyflie(scenario, scale, method_name)
        histories[method_name] = history
    fig, axes = plt.subplots(1, 2, figsize=(12.6, 5.1))
    _plot_publication_3d(fig, list(axes), scenario, histories, METHOD_SPECS_3D, [("x", "y"), ("x", "z")])
    fig.suptitle("Webots Crazyflie Dynamic Benchmark: Moving Sphere Crossing", y=1.08, fontsize=15)
    png_path, pdf_path = _save(fig, "figure_dynamic_webots_crazyflie")
    return {
        "title": "Webots Crazyflie dynamic benchmark",
        "scenario": scenario.name,
        "description": scenario.description,
        "png": str(png_path.relative_to(ROOT)),
        "pdf": str(pdf_path.relative_to(ROOT)),
    }


def _write_doc(entries: list[dict[str, str]]) -> None:
    lines = [
        "# Dynamic Paper Figures",
        "",
        "These are the publication-oriented trajectory figures extracted from the larger dynamic benchmark suite.",
        "",
        "Design choices:",
        "- only three algorithms are shown in each figure: `MFI-PD`, `MFI-Geometric`, and `APF`.",
        "- moving obstacles are drawn using three snapshots: initial, mid-interaction, and near-end.",
        "- dashed arrows indicate the obstacle motion direction.",
        "- the 3D cases are shown as `x-y` and `x-z` projections for readability.",
        "",
    ]
    for entry in entries:
        lines.extend(
            [
                f"## {entry['title']}",
                "",
                f"- scenario: `{entry['scenario']}`",
                f"- description: {entry['description']}",
                f"- PNG: [{Path(entry['png']).name}](</Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation/{entry['png']}>)",
                f"- PDF: [{Path(entry['pdf']).name}](</Users/ahmadataka/Documents/Bitbucket - Ataka/magnetic-field-inspired-navigation/{entry['pdf']}>)",
                "",
            ]
        )
    DOC_PATH.write_text("\n".join(lines))


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    entries = [
        _make_double_integrator_3d_figure(),
        _make_epuck_figure(),
        _make_crazyflie_figure(),
    ]
    _write_doc(entries)
    for entry in entries:
        print(f"{entry['title']}: png={entry['png']} pdf={entry['pdf']}")
    print(f"dynamic_paper_figures_doc={DOC_PATH}")


if __name__ == "__main__":
    main()
