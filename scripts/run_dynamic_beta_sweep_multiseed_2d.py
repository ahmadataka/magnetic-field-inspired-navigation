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
    DoubleIntegratorState,
    HaddadinNavigator,
    MovingCircleObstacle,
    ReferenceNavigator,
    SabattiniNavigator,
    compute_metrics,
    make_dynamic_scenarios_2d,
    make_paper_geometric_config,
    make_paper_pd_config,
    simulate,
)
from mfinav.utils.paths import benchmark_artifact_dir  # noqa: E402


ARTIFACTS = benchmark_artifact_dir(ROOT, "dynamic_beta_sweep_multiseed_2d")
RAW_CSV = ARTIFACTS / "dynamic_beta_sweep_multiseed_raw.csv"
SUMMARY_CSV = ARTIFACTS / "dynamic_beta_sweep_multiseed_summary.csv"
SUMMARY_MD = ARTIFACTS / "dynamic_beta_sweep_multiseed_summary.md"
PLOT_PNG = ARTIFACTS / "dynamic_beta_sweep_multiseed.png"

METHOD_SPECS = {
    "paper_pd": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric": {"label": "MFI-Geometric", "color": "#2ca02c"},
    "apf": {"label": "APF", "color": "#ff7f0e"},
    "haddadin": {"label": "Haddadin", "color": "#8b5cf6"},
    "sabattini": {"label": "Sabattini", "color": "#d97706"},
}

SCENARIO_NAMES = ["moving_circle_crossing", "head_on_circle"]
SEEDS = list(range(5))
OBSTACLE_SPEEDS = [0.0, 0.30, 0.60, 0.90, 1.20, 1.50, 1.80]
START_OFFSET_SCALE = 0.08
INITIAL_SPEED_SCALE = 0.05
PHASE_TIME_SHIFT_S = 0.25


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


def _scenario_with_speed(scenario, speed: float):
    updated_obstacles = []
    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, MovingCircleObstacle):
            velocity = np.asarray(obstacle.velocity, dtype=float)
            direction = velocity / max(np.linalg.norm(velocity), 1e-12)
            updated_obstacles.append(
                MovingCircleObstacle(
                    initial_center=np.asarray(obstacle.initial_center, dtype=float).copy(),
                    radius=float(obstacle.radius),
                    velocity=direction * speed,
                    oscillation_amplitude=None if obstacle.oscillation_amplitude is None else np.asarray(obstacle.oscillation_amplitude, dtype=float).copy(),
                    oscillation_frequency=None if obstacle.oscillation_frequency is None else np.asarray(obstacle.oscillation_frequency, dtype=float).copy(),
                    oscillation_phase=None if obstacle.oscillation_phase is None else np.asarray(obstacle.oscillation_phase, dtype=float).copy(),
                )
            )
        else:
            updated_obstacles.append(obstacle)
    return replace(scenario, obstacles=type(scenario.obstacles)(obstacles=updated_obstacles))


def _perturb_scenario(base_scenario, seed: int):
    rng = np.random.default_rng(seed)
    goal_dir = _unit(np.asarray(base_scenario.goal, dtype=float) - np.asarray(base_scenario.start, dtype=float))
    lateral = np.array([-goal_dir[1], goal_dir[0]], dtype=float)
    start = np.asarray(base_scenario.start, dtype=float) + START_OFFSET_SCALE * rng.normal() * lateral
    initial_velocity = abs(INITIAL_SPEED_SCALE * rng.normal()) * _rotate_2d(goal_dir, math.radians(8.0) * rng.normal())

    time_shift = PHASE_TIME_SHIFT_S * rng.normal()
    shifted_obstacles = []
    for obstacle in base_scenario.obstacles.obstacles:
        if isinstance(obstacle, MovingCircleObstacle):
            phase = None if obstacle.oscillation_phase is None else np.asarray(obstacle.oscillation_phase, dtype=float) + np.asarray(obstacle.oscillation_frequency, dtype=float) * time_shift
            shifted_obstacles.append(
                MovingCircleObstacle(
                    initial_center=np.asarray(obstacle.initial_center, dtype=float) + np.asarray(obstacle.velocity, dtype=float) * time_shift,
                    radius=float(obstacle.radius),
                    velocity=np.asarray(obstacle.velocity, dtype=float).copy(),
                    oscillation_amplitude=None if obstacle.oscillation_amplitude is None else np.asarray(obstacle.oscillation_amplitude, dtype=float).copy(),
                    oscillation_frequency=None if obstacle.oscillation_frequency is None else np.asarray(obstacle.oscillation_frequency, dtype=float).copy(),
                    oscillation_phase=phase,
                )
            )
        else:
            shifted_obstacles.append(obstacle)
    return replace(base_scenario, start=start, obstacles=type(base_scenario.obstacles)(obstacles=shifted_obstacles)), initial_velocity


def _simulate_methods(scenario, initial_velocity: np.ndarray, config_pd, config_geometric):
    histories = {
        "paper_pd": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_pd, navigator=ReferenceNavigator(config_pd)),
        "paper_geometric": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_geometric, navigator=ReferenceNavigator(config_geometric)),
        "apf": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_pd, navigator=ArtificialPotentialFieldNavigator(config_pd)),
        "haddadin": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_pd, navigator=HaddadinNavigator(config_pd)),
        "sabattini": simulate(_initial_state(scenario.start, initial_velocity), scenario.goal, scenario.obstacles, config_pd, navigator=SabattiniNavigator(config_pd)),
    }
    return {name: compute_metrics(history, scenario.goal) for name, history in histories.items()}


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


def _summarize(raw_rows: list[dict[str, str | float]]) -> list[dict[str, str | float]]:
    grouped: dict[tuple[str, str, float], list[dict[str, str | float]]] = {}
    for row in raw_rows:
        grouped.setdefault((str(row["scenario"]), str(row["method"]), float(row["beta_ref"])), []).append(row)

    summary_rows: list[dict[str, str | float]] = []
    for scenario_name in SCENARIO_NAMES:
        for method_name in METHOD_SPECS:
            for speed in OBSTACLE_SPEEDS:
                rows = grouped[(scenario_name, method_name, speed / 1.5)]
                success = [float(row["success"]) for row in rows]
                collision = [float(row["collision"]) for row in rows]
                safety = [float(row["safety_violation"]) for row in rows]
                final_error = [float(row["final_goal_distance"]) for row in rows]
                clearance = [float(row["min_clearance"]) for row in rows]
                summary_rows.append(
                    {
                        "scenario": scenario_name,
                        "method": method_name,
                        "method_label": METHOD_SPECS[method_name]["label"],
                        "obstacle_speed": speed,
                        "beta_ref": speed / 1.5,
                        "seeds": len(rows),
                        "success_rate_mean": statistics.fmean(success),
                        "collision_rate_mean": statistics.fmean(collision),
                        "safety_violation_rate_mean": statistics.fmean(safety),
                        "final_goal_distance_mean": _mean_std(final_error)[0],
                        "final_goal_distance_std": _mean_std(final_error)[1],
                        "min_clearance_mean": _mean_std(clearance)[0],
                        "min_clearance_std": _mean_std(clearance)[1],
                        "worst_min_clearance": min(clearance),
                    }
                )
    return summary_rows


def _plot(summary_rows: list[dict[str, str | float]]) -> None:
    metrics = [
        ("success_rate_mean", None, "success rate"),
        ("min_clearance_mean", "min_clearance_std", "min clearance [m]"),
        ("final_goal_distance_mean", "final_goal_distance_std", "final goal distance [m]"),
    ]
    fig, axes = plt.subplots(len(SCENARIO_NAMES), len(metrics), figsize=(15.5, 5.0 * len(SCENARIO_NAMES)))
    axes = np.atleast_2d(axes)
    for row_idx, scenario_name in enumerate(SCENARIO_NAMES):
        subset = [row for row in summary_rows if row["scenario"] == scenario_name]
        for col_idx, (mean_key, std_key, label) in enumerate(metrics):
            ax = axes[row_idx][col_idx]
            for method_name, spec in METHOD_SPECS.items():
                rows = [row for row in subset if row["method"] == method_name]
                rows.sort(key=lambda item: float(item["beta_ref"]))
                x = [float(row["beta_ref"]) for row in rows]
                y = [float(row[mean_key]) for row in rows]
                ax.plot(x, y, color=spec["color"], linewidth=2.0, marker="o", markersize=4.2, label=spec["label"])
                if std_key is not None:
                    std = np.asarray([float(row[std_key]) for row in rows], dtype=float)
                    y_arr = np.asarray(y, dtype=float)
                    ax.fill_between(x, y_arr - std, y_arr + std, color=spec["color"], alpha=0.12)
            ax.axvline(1.0, color="#666666", linestyle="--", linewidth=1.0, alpha=0.8)
            ax.set_title(scenario_name.replace("_", " "))
            ax.set_xlabel(r"$\beta_{\mathrm{ref}} = \nu / v_{\mathrm{ref}}$")
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.25)
            if mean_key == "success_rate_mean":
                ax.set_ylim(0.0, 1.05)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=5, frameon=False)
    fig.suptitle(r"Dynamic 2D multi-seed beta sweep with $v_{\mathrm{ref}} = 1.5$ m/s", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(PLOT_PNG, dpi=180)
    plt.close(fig)


def _write_markdown(summary_rows: list[dict[str, str | float]]) -> None:
    lines = [
        "# Dynamic Beta Sweep Multi-seed Summary",
        "",
        "This sweep repeats selected dynamic 2D circle scenarios across seeded perturbations.",
        "",
        f"- seeds: `{SEEDS}`",
        f"- obstacle speeds: `{OBSTACLE_SPEEDS}` m/s",
        "- reference speed: `v_ref = 1.5 m/s`",
        "",
        "| Scenario | Method | Beta | Success Rate | Collision Rate | Safety Viol Rate | Final Error | Min Clearance | Worst Clearance |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary_rows:
        lines.append(
            f"| {row['scenario']} | {row['method_label']} | {float(row['beta_ref']):.3f} | {float(row['success_rate_mean']):.3f} | {float(row['collision_rate_mean']):.3f} | {float(row['safety_violation_rate_mean']):.3f} | {float(row['final_goal_distance_mean']):.3f} ± {float(row['final_goal_distance_std']):.3f} | {float(row['min_clearance_mean']):.3f} ± {float(row['min_clearance_std']):.3f} | {float(row['worst_min_clearance']):.3f} |"
        )
    SUMMARY_MD.write_text("\n".join(lines) + "\n")


def main() -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    scenarios_by_name = {scenario.name: scenario for scenario in make_dynamic_scenarios_2d()}
    config_pd = make_paper_pd_config()
    config_geometric = make_paper_geometric_config()
    v_ref = float(config_pd.speed_limit)

    raw_rows: list[dict[str, str | float]] = []
    for scenario_name in SCENARIO_NAMES:
        base_nominal = scenarios_by_name[scenario_name]
        for speed in OBSTACLE_SPEEDS:
            speed_scenario = _scenario_with_speed(base_nominal, speed)
            beta_ref = speed / v_ref
            for seed in SEEDS:
                scenario, initial_velocity = _perturb_scenario(speed_scenario, seed)
                metrics_by_method = _simulate_methods(scenario, initial_velocity, config_pd, config_geometric)
                for method_name, metrics in metrics_by_method.items():
                    raw_rows.append(
                        {
                            "scenario": scenario_name,
                            "method": method_name,
                            "method_label": METHOD_SPECS[method_name]["label"],
                            "seed": seed,
                            "obstacle_speed": speed,
                            "v_ref": v_ref,
                            "beta_ref": beta_ref,
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
    print(f"dynamic_beta_sweep_multiseed_raw={RAW_CSV}")
    print(f"dynamic_beta_sweep_multiseed_summary={SUMMARY_CSV}")
    print(f"dynamic_beta_sweep_multiseed_md={SUMMARY_MD}")
    print(f"dynamic_beta_sweep_multiseed_plot={PLOT_PNG}")


if __name__ == "__main__":
    main()
