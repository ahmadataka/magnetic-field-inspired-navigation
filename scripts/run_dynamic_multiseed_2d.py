#!/usr/bin/env python3
from __future__ import annotations

import csv
from dataclasses import replace
import math
from pathlib import Path
import statistics
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
    CircleObstacle,
    DoubleIntegratorState,
    DynamicObstacleCollection,
    HaddadinNavigator,
    MovingCircleObstacle,
    MovingPolygonObstacle,
    PolygonObstacle,
    ReferenceNavigator,
    SabattiniNavigator,
    compute_metrics,
    make_dynamic_scenarios_2d,
    make_paper_geometric_config,
    make_paper_pd_config,
    simulate,
)
from mfinav.utils.paths import benchmark_artifact_dir  # noqa: E402


ARTIFACTS = benchmark_artifact_dir(ROOT, "dynamic_multiseed_2d")
RAW_CSV = ARTIFACTS / "dynamic_multiseed_raw.csv"
SUMMARY_CSV = ARTIFACTS / "dynamic_multiseed_summary.csv"
SUMMARY_MD = ARTIFACTS / "dynamic_multiseed_summary.md"
PLOT_PNG = ARTIFACTS / "dynamic_multiseed_summary.png"

METHOD_SPECS = {
    "paper_pd": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric": {"label": "MFI-Geometric", "color": "#2ca02c"},
    "apf": {"label": "APF", "color": "#ff7f0e"},
    "haddadin": {"label": "Haddadin", "color": "#8b5cf6"},
    "sabattini": {"label": "Sabattini", "color": "#d97706"},
}

SEEDS = list(range(5))
START_OFFSET_SCALE = 0.12
INITIAL_SPEED_SCALE = 0.08
PHASE_TIME_SHIFT_S = 0.35


def _initial_state(start: np.ndarray, velocity: np.ndarray) -> DoubleIntegratorState:
    return DoubleIntegratorState(position=start.copy(), velocity=velocity.copy())


def _unit(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-12:
        return np.zeros_like(vec)
    return vec / norm


def _rotate_2d(vec: np.ndarray, angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]], dtype=float)


def _shift_obstacle_in_time(obstacle, time_shift: float):
    if isinstance(obstacle, MovingCircleObstacle):
        phase = None if obstacle.oscillation_phase is None else np.asarray(obstacle.oscillation_phase, dtype=float) + np.asarray(obstacle.oscillation_frequency, dtype=float) * time_shift
        return MovingCircleObstacle(
            initial_center=np.asarray(obstacle.initial_center, dtype=float) + np.asarray(obstacle.velocity, dtype=float) * time_shift,
            radius=float(obstacle.radius),
            velocity=np.asarray(obstacle.velocity, dtype=float).copy(),
            oscillation_amplitude=None if obstacle.oscillation_amplitude is None else np.asarray(obstacle.oscillation_amplitude, dtype=float).copy(),
            oscillation_frequency=None if obstacle.oscillation_frequency is None else np.asarray(obstacle.oscillation_frequency, dtype=float).copy(),
            oscillation_phase=phase,
        )
    if isinstance(obstacle, MovingPolygonObstacle):
        phase = None if obstacle.oscillation_phase is None else np.asarray(obstacle.oscillation_phase, dtype=float) + np.asarray(obstacle.oscillation_frequency, dtype=float) * time_shift
        return MovingPolygonObstacle(
            initial_vertices=np.asarray(obstacle.initial_vertices, dtype=float) + np.asarray(obstacle.velocity, dtype=float) * time_shift,
            velocity=np.asarray(obstacle.velocity, dtype=float).copy(),
            oscillation_amplitude=None if obstacle.oscillation_amplitude is None else np.asarray(obstacle.oscillation_amplitude, dtype=float).copy(),
            oscillation_frequency=None if obstacle.oscillation_frequency is None else np.asarray(obstacle.oscillation_frequency, dtype=float).copy(),
            oscillation_phase=phase,
        )
    if isinstance(obstacle, CircleObstacle):
        return CircleObstacle(center=np.asarray(obstacle.center, dtype=float).copy(), radius=float(obstacle.radius))
    if isinstance(obstacle, PolygonObstacle):
        return PolygonObstacle(vertices=np.asarray(obstacle.vertices, dtype=float).copy())
    raise ValueError(f"Unsupported obstacle type {type(obstacle)!r}")


def _perturb_scenario(base_scenario, seed: int):
    rng = np.random.default_rng(seed)
    goal_dir = _unit(np.asarray(base_scenario.goal, dtype=float) - np.asarray(base_scenario.start, dtype=float))
    lateral = np.array([-goal_dir[1], goal_dir[0]], dtype=float)
    lateral_offset = START_OFFSET_SCALE * rng.normal()
    start = np.asarray(base_scenario.start, dtype=float) + lateral_offset * lateral

    heading_perturb = math.radians(10.0) * rng.normal()
    speed_mag = abs(INITIAL_SPEED_SCALE * rng.normal())
    velocity = speed_mag * _rotate_2d(goal_dir, heading_perturb)

    time_shift = PHASE_TIME_SHIFT_S * rng.normal()
    perturbed_obstacles = [_shift_obstacle_in_time(obstacle, time_shift) for obstacle in base_scenario.obstacles.obstacles]

    return (
        replace(base_scenario, start=start, obstacles=DynamicObstacleCollection(obstacles=perturbed_obstacles)),
        velocity,
        {
            "start_offset_lateral": float(lateral_offset),
            "initial_speed": float(speed_mag),
            "heading_perturb_rad": float(heading_perturb),
            "obstacle_time_shift_s": float(time_shift),
        },
    )


def _simulate_methods(scenario, initial_velocity: np.ndarray, config_pd, config_geometric) -> dict[str, dict[str, float]]:
    histories = {
        "paper_pd": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_pd, navigator=ReferenceNavigator(config_pd)),
        "paper_geometric": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_geometric, navigator=ReferenceNavigator(config_geometric)),
        "apf": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_pd, navigator=ArtificialPotentialFieldNavigator(config_pd)),
        "haddadin": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_pd, navigator=HaddadinNavigator(config_pd)),
        "sabattini": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_pd, navigator=SabattiniNavigator(config_pd)),
    }
    return {method_name: compute_metrics(history, scenario.goal) for method_name, history in histories.items()}


def _write_csv(path: Path, rows: list[dict[str, str | float]], fieldnames: list[str]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _mean_std(values: list[float]) -> tuple[float, float]:
    if not values:
        return math.nan, math.nan
    if len(values) == 1:
        return values[0], 0.0
    return statistics.fmean(values), statistics.stdev(values)


def _format(mean: float, std: float, digits: int = 3) -> str:
    if math.isnan(mean):
        return "-"
    return f"{mean:.{digits}f} ± {std:.{digits}f}"


def _summarize(raw_rows: list[dict[str, str | float]]) -> list[dict[str, str | float]]:
    grouped: dict[tuple[str, str], list[dict[str, str | float]]] = {}
    for row in raw_rows:
        grouped.setdefault((str(row["scenario"]), str(row["method"])), []).append(row)

    summary_rows: list[dict[str, str | float]] = []
    for scenario_name in [scenario.name for scenario in make_dynamic_scenarios_2d()]:
        for method_name in METHOD_SPECS:
            rows = grouped[(scenario_name, method_name)]
            success = [float(row["success"]) for row in rows]
            collision = [float(row["collision"]) for row in rows]
            safety = [float(row["safety_violation"]) for row in rows]
            final_error = [float(row["final_goal_distance"]) for row in rows]
            clearance = [float(row["min_clearance"]) for row in rows]
            time_to_goal = [float(row["time_to_goal_steps"]) for row in rows if math.isfinite(float(row["time_to_goal_steps"]))]
            summary_rows.append(
                {
                    "scenario": scenario_name,
                    "method": method_name,
                    "method_label": METHOD_SPECS[method_name]["label"],
                    "seeds": len(rows),
                    "success_rate_mean": statistics.fmean(success),
                    "collision_rate_mean": statistics.fmean(collision),
                    "safety_violation_rate_mean": statistics.fmean(safety),
                    "final_goal_distance_mean": _mean_std(final_error)[0],
                    "final_goal_distance_std": _mean_std(final_error)[1],
                    "min_clearance_mean": _mean_std(clearance)[0],
                    "min_clearance_std": _mean_std(clearance)[1],
                    "worst_min_clearance": min(clearance),
                    "time_to_goal_steps_mean": _mean_std(time_to_goal)[0] if time_to_goal else math.inf,
                    "time_to_goal_steps_std": _mean_std(time_to_goal)[1] if time_to_goal else math.inf,
                }
            )
    return summary_rows


def _plot(summary_rows: list[dict[str, str | float]]) -> None:
    scenario_names = [scenario.name for scenario in make_dynamic_scenarios_2d()]
    fig, axes = plt.subplots(len(scenario_names), 2, figsize=(12.5, 4.2 * len(scenario_names)))
    axes = np.atleast_2d(axes)
    for row_idx, scenario_name in enumerate(scenario_names):
        subset = [row for row in summary_rows if row["scenario"] == scenario_name]
        methods = [row["method"] for row in subset]
        x = np.arange(len(methods))
        success = [float(row["success_rate_mean"]) for row in subset]
        clearance = [float(row["min_clearance_mean"]) for row in subset]
        clearance_err = [float(row["min_clearance_std"]) for row in subset]
        colors = [METHOD_SPECS[name]["color"] for name in methods]
        axes[row_idx][0].bar(x, success, color=colors, alpha=0.9)
        axes[row_idx][0].set_ylim(0.0, 1.05)
        axes[row_idx][0].set_ylabel("success rate")
        axes[row_idx][0].set_title(scenario_name.replace("_", " "))
        axes[row_idx][0].set_xticks(x, [METHOD_SPECS[name]["label"] for name in methods], rotation=25, ha="right")
        axes[row_idx][0].grid(True, axis="y", alpha=0.25)

        axes[row_idx][1].bar(x, clearance, yerr=clearance_err, color=colors, alpha=0.9, capsize=4)
        axes[row_idx][1].set_ylabel("min clearance [m]")
        axes[row_idx][1].set_title(f"{scenario_name.replace('_', ' ')}")
        axes[row_idx][1].set_xticks(x, [METHOD_SPECS[name]["label"] for name in methods], rotation=25, ha="right")
        axes[row_idx][1].grid(True, axis="y", alpha=0.25)

    fig.suptitle("Dynamic 2D multi-seed benchmark (5 seeds)", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    fig.savefig(PLOT_PNG, dpi=180)
    plt.close(fig)


def _write_markdown(summary_rows: list[dict[str, str | float]]) -> None:
    lines = [
        "# Dynamic Multiseed 2D Summary",
        "",
        "This benchmark repeats the dynamic 2D scenarios with small seeded perturbations around the same nominal setup.",
        "",
        "Perturbations per seed:",
        "- start position lateral offset",
        "- small initial velocity / heading perturbation",
        "- small moving-obstacle time-phase shift",
        "",
        f"- seeds: `{SEEDS}`",
        f"- lateral offset scale: `{START_OFFSET_SCALE}` m",
        f"- initial speed scale: `{INITIAL_SPEED_SCALE}` m/s",
        f"- obstacle time-shift scale: `{PHASE_TIME_SHIFT_S}` s",
        "",
        "| Scenario | Method | Success Rate | Collision Rate | Safety Viol Rate | Final Error | Min Clearance | Worst Clearance | T_goal (steps) |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary_rows:
        time_mean = float(row["time_to_goal_steps_mean"])
        time_std = float(row["time_to_goal_steps_std"])
        time_text = "-" if math.isinf(time_mean) else _format(time_mean, time_std, 1)
        lines.append(
            f"| {row['scenario']} | {row['method_label']} | {float(row['success_rate_mean']):.3f} | {float(row['collision_rate_mean']):.3f} | {float(row['safety_violation_rate_mean']):.3f} | {_format(float(row['final_goal_distance_mean']), float(row['final_goal_distance_std']))} | {_format(float(row['min_clearance_mean']), float(row['min_clearance_std']))} | {float(row['worst_min_clearance']):.3f} | {time_text} |"
        )
    SUMMARY_MD.write_text("\n".join(lines) + "\n")


def main() -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    scenarios = make_dynamic_scenarios_2d()
    config_pd = make_paper_pd_config()
    config_geometric = make_paper_geometric_config()

    raw_rows: list[dict[str, str | float]] = []
    for base_scenario in scenarios:
        for seed in SEEDS:
            scenario, initial_velocity, perturbation = _perturb_scenario(base_scenario, seed)
            metrics_by_method = _simulate_methods(scenario, initial_velocity, config_pd, config_geometric)
            for method_name, metrics in metrics_by_method.items():
                raw_rows.append(
                    {
                        "scenario": base_scenario.name,
                        "method": method_name,
                        "method_label": METHOD_SPECS[method_name]["label"],
                        "seed": seed,
                        "start_offset_lateral": perturbation["start_offset_lateral"],
                        "initial_speed": perturbation["initial_speed"],
                        "heading_perturb_rad": perturbation["heading_perturb_rad"],
                        "obstacle_time_shift_s": perturbation["obstacle_time_shift_s"],
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

    summary_rows = _summarize(raw_rows)
    _write_csv(RAW_CSV, raw_rows, list(raw_rows[0].keys()))
    _write_csv(SUMMARY_CSV, summary_rows, list(summary_rows[0].keys()))
    _write_markdown(summary_rows)
    _plot(summary_rows)
    print(f"dynamic_multiseed_raw={RAW_CSV}")
    print(f"dynamic_multiseed_summary={SUMMARY_CSV}")
    print(f"dynamic_multiseed_md={SUMMARY_MD}")
    print(f"dynamic_multiseed_plot={PLOT_PNG}")


if __name__ == "__main__":
    main()
