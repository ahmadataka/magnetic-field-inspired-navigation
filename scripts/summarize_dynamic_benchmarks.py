from __future__ import annotations

import csv
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mfinav.utils.paths import benchmark_artifact_dir, summary_artifact_dir


INPUTS = [
    (
        "dynamic_double_integrator_2d",
        benchmark_artifact_dir(ROOT, "dynamic_double_integrator_2d") / "benchmark_metrics_dynamic_2d.csv",
        0.02,
        "2D double integrator",
    ),
    (
        "dynamic_differential_drive_2d",
        benchmark_artifact_dir(ROOT, "dynamic_differential_drive_2d") / "benchmark_metrics_dynamic_diff_drive_2d.csv",
        0.02,
        "2D differential drive",
    ),
    (
        "dynamic_double_integrator_3d",
        benchmark_artifact_dir(ROOT, "dynamic_double_integrator_3d") / "benchmark_metrics_dynamic_3d.csv",
        0.02,
        "3D double integrator",
    ),
    (
        "dynamic_quadrotor_3d",
        benchmark_artifact_dir(ROOT, "dynamic_quadrotor_3d") / "benchmark_metrics_dynamic_quadrotor_3d.csv",
        0.02,
        "3D quadrotor",
    ),
    (
        "webots_epuck_dynamic",
        benchmark_artifact_dir(ROOT, "webots_epuck_dynamic") / "benchmark_metrics_dynamic_webots_epuck.csv",
        0.064,
        "Webots e-puck",
    ),
    (
        "webots_crazyflie_dynamic",
        benchmark_artifact_dir(ROOT, "webots_crazyflie_dynamic") / "benchmark_metrics_webots_crazyflie_dynamic.csv",
        0.032,
        "Webots Crazyflie",
    ),
]

OUTPUT_DIR = summary_artifact_dir(ROOT, "dynamic_master")
OUTPUT_AGG_CSV = OUTPUT_DIR / "dynamic_master_summary.csv"
OUTPUT_SCENARIO_CSV = OUTPUT_DIR / "dynamic_master_per_scenario.csv"
OUTPUT_MD = DOCS / "dynamic_benchmark_master_summary.md"
OUTPUT_HTML = DOCS / "dynamic_benchmark_master_summary.html"

MODEL_ORDER = [item[0] for item in INPUTS]
MODEL_LABELS = {item[0]: item[3] for item in INPUTS}
DT_BY_MODEL = {item[0]: item[2] for item in INPUTS}
METHOD_LABELS = {
    "paper_pd": "MFI-PD",
    "paper_geometric": "MFI-Geometric",
    "apf": "APF",
    "haddadin": "Haddadin",
    "sabattini": "Sabattini",
    "paper_pd_3d": "MFI-PD",
    "paper_geometric_3d": "MFI-Geometric",
    "apf_3d": "APF",
    "haddadin_3d": "Haddadin",
    "sabattini_3d": "Sabattini",
}


def _load_rows() -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for model, path, dt, _label in INPUTS:
        with path.open() as handle:
            for row in csv.DictReader(handle):
                merged = dict(row)
                merged["model"] = model
                merged["dt"] = str(dt)
                rows.append(merged)
    return rows


def _as_float(row: dict[str, str], key: str) -> float:
    value = row[key]
    if value.lower() == "inf":
        return math.inf
    return float(value)


def _mean(values: list[float]) -> float:
    if not values:
        return math.nan
    return sum(values) / len(values)


def _format_number(value: float, digits: int = 3) -> str:
    if math.isnan(value):
        return "-"
    if math.isinf(value):
        return "inf"
    return f"{value:.{digits}f}"


def _metric_value(value: str) -> float | None:
    if value in {"-", "inf"}:
        return None if value == "-" else math.inf
    try:
        return float(value)
    except ValueError:
        return None


def _summarize(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    grouped: dict[tuple[str, str], list[dict[str, str]]] = {}
    for row in rows:
        grouped.setdefault((row["model"], row["method"]), []).append(row)

    summary: list[dict[str, str]] = []
    for (model, method), items in sorted(
        grouped.items(),
        key=lambda item: (MODEL_ORDER.index(item[0][0]), METHOD_LABELS.get(item[0][1], item[0][1])),
    ):
        success = [_as_float(row, "success") for row in items]
        reached_once = [_as_float(row, "goal_reached_once") for row in items]
        path_length = [_as_float(row, "path_length") for row in items]
        final_error = [_as_float(row, "final_goal_distance") for row in items]
        clearance = [_as_float(row, "min_clearance") for row in items]
        mean_speed = [_as_float(row, "mean_speed") for row in items]
        collision = [_as_float(row, "collision") for row in items]
        safety = [_as_float(row, "safety_violation") for row in items]
        efficiency = [_as_float(row, "path_efficiency") for row in items]
        finite_time_steps = [
            _as_float(row, "time_to_goal_steps")
            for row in items
            if math.isfinite(_as_float(row, "time_to_goal_steps"))
        ]
        dt = DT_BY_MODEL[model]
        mean_time_steps = _mean(finite_time_steps)
        mean_time_seconds = mean_time_steps * dt if finite_time_steps else math.nan
        statuses = [row.get("status", "completed") for row in items]
        status_counts = {status: statuses.count(status) for status in sorted(set(statuses))}
        dominant_status = max(status_counts.items(), key=lambda pair: pair[1])[0]
        summary.append(
            {
                "model": model,
                "model_label": MODEL_LABELS[model],
                "method": method,
                "method_label": METHOD_LABELS.get(method, method),
                "scenarios": str(len(items)),
                "success_count": str(int(round(sum(success)))),
                "success_rate": _format_number(_mean(success), 3),
                "goal_reached_once_rate": _format_number(_mean(reached_once), 3),
                "mean_path_length": _format_number(_mean(path_length), 3),
                "mean_time_to_goal_steps": _format_number(mean_time_steps, 1),
                "mean_time_to_goal_seconds": _format_number(mean_time_seconds, 2),
                "mean_final_goal_distance": _format_number(_mean(final_error), 3),
                "mean_min_clearance": _format_number(_mean(clearance), 3),
                "worst_min_clearance": _format_number(min(clearance), 3),
                "collision_rate": _format_number(_mean(collision), 3),
                "safety_violation_rate": _format_number(_mean(safety), 3),
                "mean_speed": _format_number(_mean(mean_speed), 3),
                "mean_path_efficiency": _format_number(_mean(efficiency), 3),
                "dominant_status": dominant_status,
            }
        )
    return summary


def _scenario_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    detailed: list[dict[str, str]] = []
    for row in sorted(
        rows,
        key=lambda item: (MODEL_ORDER.index(item["model"]), item["scenario"], METHOD_LABELS.get(item["method"], item["method"])),
    ):
        time_steps = _as_float(row, "time_to_goal_steps")
        dt = float(row["dt"])
        detailed.append(
            {
                "model": row["model"],
                "model_label": MODEL_LABELS[row["model"]],
                "scenario": row["scenario"],
                "method": row["method"],
                "method_label": METHOD_LABELS.get(row["method"], row["method"]),
                "status": row.get("status", "completed"),
                "success": _format_number(_as_float(row, "success"), 3),
                "goal_reached_once": _format_number(_as_float(row, "goal_reached_once"), 3),
                "path_length": _format_number(_as_float(row, "path_length"), 3),
                "time_to_goal_steps": _format_number(time_steps, 1),
                "time_to_goal_seconds": _format_number(time_steps * dt, 2),
                "final_goal_distance": _format_number(_as_float(row, "final_goal_distance"), 3),
                "min_clearance": _format_number(_as_float(row, "min_clearance"), 3),
                "collision": _format_number(_as_float(row, "collision"), 3),
                "safety_violation": _format_number(_as_float(row, "safety_violation"), 3),
                "path_efficiency": _format_number(_as_float(row, "path_efficiency"), 3),
            }
        )
    return detailed


def _write_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _render_table(title: str, rows: list[dict[str, str]], columns: list[tuple[str, str]], metric_rules: dict[str, str]) -> list[str]:
    lines = [f"<section class=\"table-section\">", f"<h3>{title}</h3>", "<div class=\"table-wrap\">", "<table>", "<thead>", "<tr>"]
    for _, label in columns:
        lines.append(f"<th>{label}</th>")
    lines.extend(["</tr>", "</thead>", "<tbody>"])

    best_worst: dict[str, tuple[float | None, float | None]] = {}
    for key, rule in metric_rules.items():
        values = []
        for row in rows:
            numeric = _metric_value(row[key])
            if numeric is None or math.isinf(numeric):
                continue
            values.append(numeric)
        if not values:
            best_worst[key] = (None, None)
        elif rule == "high":
            best_worst[key] = (max(values), min(values))
        else:
            best_worst[key] = (min(values), max(values))

    for row in rows:
        lines.append("<tr>")
        for key, _ in columns:
            value = row[key]
            classes: list[str] = []
            if key in metric_rules:
                numeric = _metric_value(value)
                best, worst = best_worst[key]
                if numeric is not None and not math.isinf(numeric) and best is not None and abs(numeric - best) < 1e-9:
                    classes.append("best")
                if numeric is not None and not math.isinf(numeric) and worst is not None and abs(numeric - worst) < 1e-9:
                    classes.append("worst")
            class_attr = f" class=\"{' '.join(classes)}\"" if classes else ""
            lines.append(f"<td{class_attr}>{value}</td>")
        lines.append("</tr>")
    lines.extend(["</tbody>", "</table>", "</div>", "</section>"])
    return lines


def _write_markdown(summary_rows: list[dict[str, str]]) -> None:
    lines = [
        "# Dynamic Benchmark Master Summary",
        "",
        "This file consolidates the currently available dynamic-environment benchmarks across the clean Python repo and Webots validation suites.",
        "",
        "Included benchmark families:",
        "- `dynamic_double_integrator_2d`",
        "- `dynamic_differential_drive_2d`",
        "- `dynamic_double_integrator_3d`",
        "- `dynamic_quadrotor_3d`",
        "- `webots_epuck_dynamic`",
        "- `webots_crazyflie_dynamic`",
        "",
        "Metric interpretation:",
        "- `success_rate`: final-state success under the suite safety rule",
        "- `goal_reached_once_rate`: fraction of runs that entered the goal region at least once without a safety violation",
        "- `mean_time_to_goal_steps` and `mean_time_to_goal_seconds`: averaged over finite goal-reaching runs only",
        "- `mean_min_clearance` and `worst_min_clearance`: obstacle-avoidance margin indicators",
        "- `collision_rate` and `safety_violation_rate`: failure indicators",
        "",
        "| Model | Method | N | Succ | Succ Rate | Reach Once | Path | T_goal (steps) | T_goal (s) | Final Err | Mean Clr | Worst Clr | Coll Rate | Safe Viol | Eff | Status |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in summary_rows:
        lines.append(
            "| {model_label} | {method_label} | {scenarios} | {success_count} | {success_rate} | {goal_reached_once_rate} | {mean_path_length} | {mean_time_to_goal_steps} | {mean_time_to_goal_seconds} | {mean_final_goal_distance} | {mean_min_clearance} | {worst_min_clearance} | {collision_rate} | {safety_violation_rate} | {mean_path_efficiency} | {dominant_status} |".format(
                **row
            )
        )
    OUTPUT_MD.write_text("\n".join(lines) + "\n")


def _write_html(summary_rows: list[dict[str, str]]) -> None:
    aggregate_columns = [
        ("method_label", "Method"),
        ("scenarios", "N"),
        ("success_count", "Succ"),
        ("success_rate", "Succ Rate"),
        ("goal_reached_once_rate", "Reach Once"),
        ("mean_path_length", "Path"),
        ("mean_time_to_goal_seconds", "T_goal (s)"),
        ("mean_final_goal_distance", "Final Err"),
        ("mean_min_clearance", "Mean Clr"),
        ("worst_min_clearance", "Worst Clr"),
        ("collision_rate", "Coll Rate"),
        ("safety_violation_rate", "Safe Viol"),
        ("mean_path_efficiency", "Eff"),
        ("dominant_status", "Status"),
    ]
    metric_rules = {
        "success_count": "high",
        "success_rate": "high",
        "goal_reached_once_rate": "high",
        "mean_path_length": "low",
        "mean_time_to_goal_seconds": "low",
        "mean_final_goal_distance": "low",
        "mean_min_clearance": "high",
        "worst_min_clearance": "high",
        "collision_rate": "low",
        "safety_violation_rate": "low",
        "mean_path_efficiency": "high",
    }

    sections: list[str] = []
    for model in MODEL_ORDER:
        model_rows = [row for row in summary_rows if row["model"] == model]
        sections.extend(_render_table(MODEL_LABELS[model], model_rows, aggregate_columns, metric_rules))

    html = [
        "<!DOCTYPE html>",
        "<html lang=\"en\">",
        "<head>",
        "  <meta charset=\"utf-8\" />",
        "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />",
        "  <title>Dynamic Benchmark Master Summary</title>",
        "  <style>",
        "    :root { color-scheme: dark; --bg: #111315; --panel: #171a1d; --grid: #2b3036; --text: #e8eaed; --muted: #aeb4bb; --best: #12351f; --worst: #3a1919; }",
        "    body { margin: 0; padding: 32px; font-family: Helvetica, Arial, sans-serif; background: var(--bg); color: var(--text); }",
        "    main { max-width: 1400px; margin: 0 auto; }",
        "    h1, h2, h3 { margin: 0 0 12px; }",
        "    p, li { color: var(--muted); line-height: 1.45; }",
        "    .table-section { margin-top: 28px; }",
        "    .table-wrap { overflow-x: auto; border: 1px solid var(--grid); border-radius: 14px; background: var(--panel); }",
        "    table { width: 100%; border-collapse: collapse; min-width: 980px; }",
        "    th, td { padding: 10px 12px; border-bottom: 1px solid var(--grid); text-align: left; vertical-align: top; }",
        "    th { position: sticky; top: 0; background: #1c2024; }",
        "    td.best { background: var(--best); }",
        "    td.worst { background: var(--worst); }",
        "    code { color: #d2e3fc; }",
        "  </style>",
        "</head>",
        "<body>",
        "  <main>",
        "    <h1>Dynamic Benchmark Master Summary</h1>",
        "    <p>This page consolidates the dynamic-environment benchmark results currently available in the clean Python and Webots validation suites.</p>",
        "    <p>Families included: <code>dynamic_double_integrator_2d</code>, <code>dynamic_differential_drive_2d</code>, <code>dynamic_double_integrator_3d</code>, <code>dynamic_quadrotor_3d</code>, <code>webots_epuck_dynamic</code>, and <code>webots_crazyflie_dynamic</code>.</p>",
        "    <p>Best and worst values are highlighted inside each model section.</p>",
        *sections,
        "  </main>",
        "</body>",
        "</html>",
    ]
    OUTPUT_HTML.write_text("\n".join(html) + "\n")


def main() -> None:
    rows = _load_rows()
    summary_rows = _summarize(rows)
    scenario_rows = _scenario_rows(rows)

    _write_csv(
        OUTPUT_AGG_CSV,
        summary_rows,
        [
            "model",
            "model_label",
            "method",
            "method_label",
            "scenarios",
            "success_count",
            "success_rate",
            "goal_reached_once_rate",
            "mean_path_length",
            "mean_time_to_goal_steps",
            "mean_time_to_goal_seconds",
            "mean_final_goal_distance",
            "mean_min_clearance",
            "worst_min_clearance",
            "collision_rate",
            "safety_violation_rate",
            "mean_speed",
            "mean_path_efficiency",
            "dominant_status",
        ],
    )
    _write_csv(
        OUTPUT_SCENARIO_CSV,
        scenario_rows,
        [
            "model",
            "model_label",
            "scenario",
            "method",
            "method_label",
            "status",
            "success",
            "goal_reached_once",
            "path_length",
            "time_to_goal_steps",
            "time_to_goal_seconds",
            "final_goal_distance",
            "min_clearance",
            "collision",
            "safety_violation",
            "path_efficiency",
        ],
    )
    _write_markdown(summary_rows)
    _write_html(summary_rows)

    print(f"dynamic_master_summary_csv={OUTPUT_AGG_CSV}")
    print(f"dynamic_master_per_scenario_csv={OUTPUT_SCENARIO_CSV}")
    print(f"dynamic_master_summary_md={OUTPUT_MD}")
    print(f"dynamic_master_summary_html={OUTPUT_HTML}")


if __name__ == "__main__":
    main()
