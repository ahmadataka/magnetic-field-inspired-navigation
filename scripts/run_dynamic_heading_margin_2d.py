#!/usr/bin/env python3
from __future__ import annotations

import csv
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
    ReferenceNavigator,
    SabattiniNavigator,
    compute_metrics,
    make_dynamic_scenarios_2d,
    make_paper_geometric_config,
    make_paper_pd_config,
    simulate,
)
from mfinav.utils.paths import benchmark_artifact_dir  # noqa: E402


ARTIFACTS = benchmark_artifact_dir(ROOT, "dynamic_heading_margin_2d")
RAW_CSV = ARTIFACTS / "dynamic_heading_margin_raw.csv"
SUMMARY_CSV = ARTIFACTS / "dynamic_heading_margin_summary.csv"
SUMMARY_MD = ARTIFACTS / "dynamic_heading_margin_summary.md"
PLOT_PNG = ARTIFACTS / "dynamic_heading_margin.png"

METHOD_SPECS = {
    "paper_pd": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric": {"label": "MFI-Geometric", "color": "#2ca02c"},
    "apf": {"label": "APF", "color": "#ff7f0e"},
    "haddadin": {"label": "Haddadin", "color": "#8b5cf6"},
    "sabattini": {"label": "Sabattini", "color": "#d97706"},
}

SCENARIO_NAMES = ["moving_circle_crossing", "head_on_circle"]
SEEDS = list(range(5))
HEADING_MARGIN_DEG = [-60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0]
START_OFFSET_SCALE = 0.06
INITIAL_SPEED_MAG = 0.80
OBSTACLE_TIME_SHIFT_S = 0.20


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


def _perturb_scenario_for_margin(base_scenario, heading_margin_deg: float, seed: int):
    rng = np.random.default_rng(seed)
    goal_dir = _unit(np.asarray(base_scenario.goal, dtype=float) - np.asarray(base_scenario.start, dtype=float))
    lateral = np.array([-goal_dir[1], goal_dir[0]], dtype=float)
    start = np.asarray(base_scenario.start, dtype=float) + START_OFFSET_SCALE * rng.normal() * lateral
    heading_rad = math.radians(heading_margin_deg)
    velocity = INITIAL_SPEED_MAG * _rotate_2d(goal_dir, heading_rad)

    time_shift = OBSTACLE_TIME_SHIFT_S * rng.normal()
    shifted_obstacles = []
    for obstacle in base_scenario.obstacles.obstacles:
        if hasattr(obstacle, "initial_center") and hasattr(obstacle, "velocity"):
            phase = None
            if getattr(obstacle, "oscillation_phase", None) is not None:
                phase = np.asarray(obstacle.oscillation_phase, dtype=float) + np.asarray(obstacle.oscillation_frequency, dtype=float) * time_shift
            shifted_obstacles.append(
                type(obstacle)(
                    **{
                        "initial_center": np.asarray(obstacle.initial_center, dtype=float) + np.asarray(obstacle.velocity, dtype=float) * time_shift,
                        "radius": float(obstacle.radius),
                        "velocity": np.asarray(obstacle.velocity, dtype=float).copy(),
                        "oscillation_amplitude": None if getattr(obstacle, "oscillation_amplitude", None) is None else np.asarray(obstacle.oscillation_amplitude, dtype=float).copy(),
                        "oscillation_frequency": None if getattr(obstacle, "oscillation_frequency", None) is None else np.asarray(obstacle.oscillation_frequency, dtype=float).copy(),
                        "oscillation_phase": phase,
                    }
                )
            )
        else:
            shifted_obstacles.append(obstacle)

    scenario = type(base_scenario)(
        name=base_scenario.name,
        start=start,
        goal=np.asarray(base_scenario.goal, dtype=float).copy(),
        obstacles=type(base_scenario.obstacles)(obstacles=shifted_obstacles),
        description=base_scenario.description,
    )
    return scenario, velocity


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


def _format(mean: float, std: float, digits: int = 3) -> str:
    if math.isnan(mean):
        return "-"
    return f"{mean:.{digits}f} ± {std:.{digits}f}"


def _summarize(raw_rows: list[dict[str, str | float]]) -> list[dict[str, str | float]]:
    grouped: dict[tuple[str, str, float], list[dict[str, str | float]]] = {}
    for row in raw_rows:
        grouped.setdefault((str(row["scenario"]), str(row["method"]), float(row["heading_margin_deg"])), []).append(row)

    summary_rows: list[dict[str, str | float]] = []
    for scenario_name in SCENARIO_NAMES:
        for method_name in METHOD_SPECS:
            for heading_margin_deg in HEADING_MARGIN_DEG:
                rows = grouped[(scenario_name, method_name, heading_margin_deg)]
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
                        "heading_margin_deg": heading_margin_deg,
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
                rows.sort(key=lambda item: float(item["heading_margin_deg"]))
                x = [float(row["heading_margin_deg"]) for row in rows]
                y = [float(row[mean_key]) for row in rows]
                ax.plot(x, y, color=spec["color"], linewidth=2.0, marker="o", markersize=4.0, label=spec["label"])
                if std_key is not None:
                    std = np.asarray([float(row[std_key]) for row in rows], dtype=float)
                    y_arr = np.asarray(y, dtype=float)
                    ax.fill_between(x, y_arr - std, y_arr + std, color=spec["color"], alpha=0.12)
            ax.axvline(0.0, color="#666666", linestyle="--", linewidth=1.0, alpha=0.8)
            ax.set_title(scenario_name.replace("_", " "))
            ax.set_xlabel("heading margin [deg]")
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.25)
            if mean_key == "success_rate_mean":
                ax.set_ylim(0.0, 1.05)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=5, frameon=False)
    fig.suptitle("Dynamic 2D heading-margin sensitivity study", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(PLOT_PNG, dpi=180)
    plt.close(fig)


def _write_markdown(summary_rows: list[dict[str, str | float]]) -> None:
    lines = [
        "# Dynamic Heading Margin 2D Summary",
        "",
        "This study sweeps the robot's initial heading margin relative to the nominal goal direction.",
        "",
        f"- scenarios: `{SCENARIO_NAMES}`",
        f"- seeds: `{SEEDS}`",
        f"- heading margins: `{HEADING_MARGIN_DEG}` deg",
        f"- initial speed magnitude: `{INITIAL_SPEED_MAG}` m/s",
        f"- start lateral offset scale: `{START_OFFSET_SCALE}` m",
        f"- obstacle time-shift scale: `{OBSTACLE_TIME_SHIFT_S}` s",
        "",
        "| Scenario | Method | Heading Margin [deg] | Success Rate | Collision Rate | Safety Viol Rate | Final Error | Min Clearance | Worst Clearance | T_goal (steps) |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary_rows:
        time_mean = float(row["time_to_goal_steps_mean"])
        time_std = float(row["time_to_goal_steps_std"])
        time_text = "-" if math.isinf(time_mean) else _format(time_mean, time_std, 1)
        lines.append(
            f"| {row['scenario']} | {row['method_label']} | {float(row['heading_margin_deg']):.1f} | {float(row['success_rate_mean']):.3f} | {float(row['collision_rate_mean']):.3f} | {float(row['safety_violation_rate_mean']):.3f} | {_format(float(row['final_goal_distance_mean']), float(row['final_goal_distance_std']))} | {_format(float(row['min_clearance_mean']), float(row['min_clearance_std']))} | {float(row['worst_min_clearance']):.3f} | {time_text} |"
        )
    SUMMARY_MD.write_text("\n".join(lines) + "\n")


def main() -> None:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    scenarios_by_name = {scenario.name: scenario for scenario in make_dynamic_scenarios_2d()}
    config_pd = make_paper_pd_config()
    config_geometric = make_paper_geometric_config()

    raw_rows: list[dict[str, str | float]] = []
    for scenario_name in SCENARIO_NAMES:
        base_scenario = scenarios_by_name[scenario_name]
        for heading_margin_deg in HEADING_MARGIN_DEG:
            for seed in SEEDS:
                scenario, initial_velocity = _perturb_scenario_for_margin(base_scenario, heading_margin_deg, seed)
                metrics_by_method = _simulate_methods(scenario, initial_velocity, config_pd, config_geometric)
                for method_name, metrics in metrics_by_method.items():
                    raw_rows.append(
                        {
                            "model_family": "dynamic_double_integrator_2d",
                            "scenario": scenario_name,
                            "method": method_name,
                            "seed": seed,
                            "heading_margin_deg": heading_margin_deg,
                            "initial_speed_mag": INITIAL_SPEED_MAG,
                            "success": metrics["success"],
                            "goal_reached_once": metrics["goal_reached_once"],
                            "collision": metrics["collision"],
                            "safety_violation": metrics["safety_violation"],
                            "min_clearance": metrics["min_clearance"],
                            "final_goal_distance": metrics["final_goal_distance"],
                            "time_to_goal_steps": metrics["time_to_goal_steps"],
                            "path_length": metrics["path_length"],
                            "path_efficiency": metrics["path_efficiency"],
                            "mean_speed": metrics["mean_speed"],
                        }
                    )

    _write_csv(
        RAW_CSV,
        raw_rows,
        [
            "model_family",
            "scenario",
            "method",
            "seed",
            "heading_margin_deg",
            "initial_speed_mag",
            "success",
            "goal_reached_once",
            "collision",
            "safety_violation",
            "min_clearance",
            "final_goal_distance",
            "time_to_goal_steps",
            "path_length",
            "path_efficiency",
            "mean_speed",
        ],
    )

    summary_rows = _summarize(raw_rows)
    _write_csv(
        SUMMARY_CSV,
        summary_rows,
        [
            "scenario",
            "method",
            "method_label",
            "heading_margin_deg",
            "seeds",
            "success_rate_mean",
            "collision_rate_mean",
            "safety_violation_rate_mean",
            "final_goal_distance_mean",
            "final_goal_distance_std",
            "min_clearance_mean",
            "min_clearance_std",
            "worst_min_clearance",
            "time_to_goal_steps_mean",
            "time_to_goal_steps_std",
        ],
    )
    _write_markdown(summary_rows)
    _plot(summary_rows)

    print(f"dynamic_heading_margin_raw={RAW_CSV}")
    print(f"dynamic_heading_margin_summary_csv={SUMMARY_CSV}")
    print(f"dynamic_heading_margin_summary_md={SUMMARY_MD}")
    print(f"dynamic_heading_margin_plot={PLOT_PNG}")


if __name__ == "__main__":
    main()
