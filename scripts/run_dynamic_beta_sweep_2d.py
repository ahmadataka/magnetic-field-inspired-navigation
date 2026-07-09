#!/usr/bin/env python3
from __future__ import annotations

import csv
from dataclasses import replace
import math
from pathlib import Path
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


ARTIFACTS = benchmark_artifact_dir(ROOT, "dynamic_beta_sweep_2d")
RAW_CSV = ARTIFACTS / "dynamic_beta_sweep_raw.csv"
SUMMARY_CSV = ARTIFACTS / "dynamic_beta_sweep_summary.csv"
PLOT_PNG = ARTIFACTS / "dynamic_beta_sweep.png"
SUMMARY_MD = ARTIFACTS / "dynamic_beta_sweep_summary.md"

METHOD_SPECS = {
    "paper_pd": {"label": "MFI-PD", "color": "#1f77b4"},
    "paper_geometric": {"label": "MFI-Geometric", "color": "#2ca02c"},
    "apf": {"label": "APF", "color": "#ff7f0e"},
    "haddadin": {"label": "Haddadin", "color": "#8b5cf6"},
    "sabattini": {"label": "Sabattini", "color": "#d97706"},
}

SCENARIO_NAMES = ["moving_circle_crossing", "head_on_circle"]
OBSTACLE_SPEEDS = [0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.05, 1.20, 1.35, 1.50]


def _initial_state(start: np.ndarray) -> DoubleIntegratorState:
    return DoubleIntegratorState(position=start.copy(), velocity=np.array([0.0, 0.0], dtype=float))


def _find_target_moving_circle(scenario):
    for obstacle in scenario.obstacles.obstacles:
        if isinstance(obstacle, MovingCircleObstacle):
            return obstacle
    raise ValueError(f"Scenario {scenario.name} does not contain a moving circle obstacle for beta sweep.")


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


def _simulate_methods(scenario, config_pd, config_geometric) -> dict[str, dict[str, float]]:
    histories = {
        "paper_pd": simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            config_pd,
            navigator=ReferenceNavigator(config_pd),
        ),
        "paper_geometric": simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            config_geometric,
            navigator=ReferenceNavigator(config_geometric),
        ),
        "apf": simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            config_pd,
            navigator=ArtificialPotentialFieldNavigator(config_pd),
        ),
        "haddadin": simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            config_pd,
            navigator=HaddadinNavigator(config_pd),
        ),
        "sabattini": simulate(
            _initial_state(scenario.start),
            scenario.goal,
            scenario.obstacles,
            config_pd,
            navigator=SabattiniNavigator(config_pd),
        ),
    }
    return {method: compute_metrics(history, scenario.goal) for method, history in histories.items()}


def _write_csv(path: Path, rows: list[dict[str, str | float]], fieldnames: list[str]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _plot(raw_rows: list[dict[str, str | float]]) -> None:
    scenarios = [name for name in SCENARIO_NAMES]
    metrics = [
        ("success", "Success", (0.0, 1.05)),
        ("final_goal_distance", "Final Goal Distance", None),
        ("min_clearance", "Min Clearance", None),
    ]
    fig, axes = plt.subplots(len(scenarios), len(metrics), figsize=(15.5, 5.1 * len(scenarios)))
    axes = np.atleast_2d(axes)

    for row_idx, scenario_name in enumerate(scenarios):
        scenario_rows = [row for row in raw_rows if row["scenario"] == scenario_name]
        for col_idx, (metric_key, metric_label, y_limits) in enumerate(metrics):
            ax = axes[row_idx][col_idx]
            for method_name, spec in METHOD_SPECS.items():
                method_rows = [row for row in scenario_rows if row["method"] == method_name]
                method_rows.sort(key=lambda item: float(item["beta_ref"]))
                ax.plot(
                    [float(row["beta_ref"]) for row in method_rows],
                    [float(row[metric_key]) for row in method_rows],
                    color=spec["color"],
                    linewidth=2.0,
                    marker="o",
                    markersize=4.2,
                    label=spec["label"],
                )
            ax.axvline(1.0, color="#666666", linestyle="--", linewidth=1.0, alpha=0.8)
            ax.set_xlabel(r"$\beta_{\mathrm{ref}} = \nu / v_{\mathrm{ref}}$")
            ax.set_ylabel(metric_label)
            ax.set_title(f"{scenario_name.replace('_', ' ')}")
            ax.grid(True, alpha=0.25)
            if y_limits is not None:
                ax.set_ylim(*y_limits)

    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=5, frameon=False)
    fig.suptitle(r"Dynamic 2D beta sweep with $v_{\mathrm{ref}} = 1.5$ m/s", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(PLOT_PNG, dpi=180)
    plt.close(fig)


def _summarize(raw_rows: list[dict[str, str | float]]) -> list[dict[str, str | float]]:
    summary_rows: list[dict[str, str | float]] = []
    for scenario_name in SCENARIO_NAMES:
        for method_name in METHOD_SPECS:
            method_rows = [row for row in raw_rows if row["scenario"] == scenario_name and row["method"] == method_name]
            method_rows.sort(key=lambda item: float(item["beta_ref"]))
            successful = [row for row in method_rows if float(row["success"]) >= 0.5]
            safe = [row for row in method_rows if float(row["collision"]) < 0.5 and float(row["safety_violation"]) < 0.5]
            summary_rows.append(
                {
                    "scenario": scenario_name,
                    "method": method_name,
                    "method_label": METHOD_SPECS[method_name]["label"],
                    "max_success_beta_ref": "-" if not successful else f"{max(float(row['beta_ref']) for row in successful):.3f}",
                    "max_safe_beta_ref": "-" if not safe else f"{max(float(row['beta_ref']) for row in safe):.3f}",
                    "best_final_goal_distance": f"{min(float(row['final_goal_distance']) for row in method_rows):.3f}",
                    "worst_min_clearance": f"{min(float(row['min_clearance']) for row in method_rows):.3f}",
                }
            )
    return summary_rows


def _write_markdown(summary_rows: list[dict[str, str | float]]) -> None:
    lines = [
        "# Dynamic Beta Sweep Summary",
        "",
        "This sweep varies obstacle speed in selected dynamic 2D scenarios and reports results against a fixed reference robot speed.",
        "",
        "- Reference speed: `v_ref = 1.5 m/s`",
        "- Sweep ratio: `beta_ref = nu / v_ref`",
        "- Scenarios: `moving_circle_crossing`, `head_on_circle`",
        "",
        "| Scenario | Method | Max Success Beta | Max Safe Beta | Best Final Err | Worst Min Clearance |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for row in summary_rows:
        lines.append(
            "| {scenario} | {method_label} | {max_success_beta_ref} | {max_safe_beta_ref} | {best_final_goal_distance} | {worst_min_clearance} |".format(
                **row
            )
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
        base_scenario = scenarios_by_name[scenario_name]
        for speed in OBSTACLE_SPEEDS:
            scenario = _scenario_with_speed(base_scenario, speed)
            beta_ref = speed / v_ref
            metrics_by_method = _simulate_methods(scenario, config_pd, config_geometric)
            for method_name, metrics in metrics_by_method.items():
                raw_rows.append(
                    {
                        "scenario": scenario_name,
                        "method": method_name,
                        "method_label": METHOD_SPECS[method_name]["label"],
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
    _write_csv(
        RAW_CSV,
        raw_rows,
        [
            "scenario",
            "method",
            "method_label",
            "obstacle_speed",
            "v_ref",
            "beta_ref",
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
    _write_csv(
        SUMMARY_CSV,
        summary_rows,
        [
            "scenario",
            "method",
            "method_label",
            "max_success_beta_ref",
            "max_safe_beta_ref",
            "best_final_goal_distance",
            "worst_min_clearance",
        ],
    )
    _plot(raw_rows)
    _write_markdown(summary_rows)

    print(f"dynamic_beta_sweep_raw={RAW_CSV}")
    print(f"dynamic_beta_sweep_summary={SUMMARY_CSV}")
    print(f"dynamic_beta_sweep_plot={PLOT_PNG}")
    print(f"dynamic_beta_sweep_md={SUMMARY_MD}")


if __name__ == "__main__":
    main()
