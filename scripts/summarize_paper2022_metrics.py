from __future__ import annotations

import csv
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ARTIFACTS = ROOT / "artifacts"
DOCS = ROOT / "docs"
DEFAULT_DT = 0.02
OUTPUT_HTML = DOCS / "paper2022_metric_summary.html"

INPUTS = [
    ("double_integrator_2d", ARTIFACTS / "benchmark_metrics.csv"),
    ("double_integrator_3d", ARTIFACTS / "benchmark_metrics_3d.csv"),
    ("differential_drive_2d", ARTIFACTS / "benchmark_metrics_diff_drive.csv"),
    ("quadrotor_3d", ARTIFACTS / "benchmark_metrics_quadrotor_3d.csv"),
]

OUTPUT_CSV = ARTIFACTS / "paper2022_metric_summary_all_models.csv"
OUTPUT_MD = DOCS / "paper2022_metric_summary.md"


def _load_rows() -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for model, path in INPUTS:
        with path.open() as handle:
            for row in csv.DictReader(handle):
                row = dict(row)
                row["model"] = model
                rows.append(row)
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


def _summarize(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    grouped: dict[tuple[str, str], list[dict[str, str]]] = {}
    for row in rows:
        grouped.setdefault((row["model"], row["method"]), []).append(row)

    summary: list[dict[str, str]] = []
    for (model, method), items in sorted(grouped.items()):
        count = len(items)
        success = [_as_float(row, "success") for row in items]
        reached_once = [_as_float(row, "goal_reached_once") for row in items]
        path_length = [_as_float(row, "path_length") for row in items]
        final_error = [_as_float(row, "final_goal_distance") for row in items]
        clearance = [_as_float(row, "min_clearance") for row in items]
        mean_speed = [_as_float(row, "mean_speed") for row in items]
        collision = [_as_float(row, "collision") for row in items]
        safety = [_as_float(row, "safety_violation") for row in items]
        efficiency = [_as_float(row, "path_efficiency") for row in items]
        finite_time = [
            _as_float(row, "time_to_goal_steps")
            for row in items
            if math.isfinite(_as_float(row, "time_to_goal_steps"))
        ]

        summary.append(
            {
                "model": model,
                "method": method,
                "scenarios": str(count),
                "success_count": str(int(round(sum(success)))),
                "success_rate": _format_number(_mean(success), 3),
                "goal_reached_once_rate": _format_number(_mean(reached_once), 3),
                "mean_path_length": _format_number(_mean(path_length), 3),
                "mean_time_to_goal_steps": _format_number(_mean(finite_time), 1),
                "mean_time_to_goal_seconds": _format_number(_mean(finite_time) * DEFAULT_DT, 2),
                "mean_final_goal_distance": _format_number(_mean(final_error), 3),
                "mean_min_clearance": _format_number(_mean(clearance), 3),
                "worst_min_clearance": _format_number(min(clearance), 3),
                "collision_rate": _format_number(_mean(collision), 3),
                "safety_violation_rate": _format_number(_mean(safety), 3),
                "mean_speed": _format_number(_mean(mean_speed), 3),
                "mean_path_efficiency": _format_number(_mean(efficiency), 3),
            }
        )
    return summary


def _scenario_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    detailed: list[dict[str, str]] = []
    for row in sorted(rows, key=lambda item: (item["model"], item["scenario"], item["method"])):
        time_steps = _as_float(row, "time_to_goal_steps")
        detailed.append(
            {
                "model": row["model"],
                "scenario": row["scenario"],
                "method": row["method"],
                "success": _format_number(_as_float(row, "success"), 3),
                "goal_reached_once": _format_number(_as_float(row, "goal_reached_once"), 3),
                "path_length": _format_number(_as_float(row, "path_length"), 3),
                "time_to_goal_steps": _format_number(time_steps, 1),
                "time_to_goal_seconds": _format_number(time_steps * DEFAULT_DT, 2),
                "final_goal_distance": _format_number(_as_float(row, "final_goal_distance"), 3),
                "min_clearance": _format_number(_as_float(row, "min_clearance"), 3),
                "collision": _format_number(_as_float(row, "collision"), 3),
                "safety_violation": _format_number(_as_float(row, "safety_violation"), 3),
                "path_efficiency": _format_number(_as_float(row, "path_efficiency"), 3),
            }
        )
    return detailed


def _write_csv(rows: list[dict[str, str]]) -> None:
    fieldnames = [
        "model",
        "method",
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
    ]
    with OUTPUT_CSV.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_markdown(rows: list[dict[str, str]]) -> None:
    lines = [
        "# Paper 2022 Metric Summary Across All Benchmarks",
        "",
        "This table consolidates the benchmark metrics already computed by the clean Python repo",
        "into a single paper-style summary across all current robot models and navigation methods.",
        "",
        "Metric interpretation:",
        "- `success_rate`: final-state success rate under the repo success radius and safety rule",
        "- `goal_reached_once_rate`: fraction of runs that entered the goal region at least once without a safety violation",
        "- `mean_path_length`: average travelled distance",
        "- `mean_time_to_goal_steps` / `mean_time_to_goal_seconds`: average first-hit time over finite goal-reaching runs only",
        "- `mean_final_goal_distance`: average final goal error, used here as an accuracy proxy",
        "- `mean_min_clearance` / `worst_min_clearance`: obstacle-avoidance margin summary",
        "- `collision_rate` / `safety_violation_rate`: hard and soft obstacle-avoidance failure rates",
        "- `mean_path_efficiency`: straight-line displacement divided by travelled path length",
        "",
        f"All benchmark scripts currently use a common simulation step of `dt = {DEFAULT_DT:.2f} s`.",
        "",
        "| Model | Method | N | Succ | Succ Rate | Reach Once | Path | T_goal (steps) | T_goal (s) | Final Err | Mean Clr | Worst Clr | Coll Rate | Safe Viol | Mean Speed | Eff |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| {model} | {method} | {scenarios} | {success_count} | {success_rate} | {goal_reached_once_rate} | {mean_path_length} | {mean_time_to_goal_steps} | {mean_time_to_goal_seconds} | {mean_final_goal_distance} | {mean_min_clearance} | {worst_min_clearance} | {collision_rate} | {safety_violation_rate} | {mean_speed} | {mean_path_efficiency} |".format(
                **row
            )
        )

    OUTPUT_MD.write_text("\n".join(lines) + "\n")


def _metric_value(value: str) -> float | None:
    if value == "-":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _render_table(title: str, rows: list[dict[str, str]], columns: list[tuple[str, str]], metric_rules: dict[str, str]) -> list[str]:
    lines = [f"<section class=\"table-section\">", f"<h3>{title}</h3>", "<div class=\"table-wrap\">", "<table>", "<thead>", "<tr>"]
    for _, label in columns:
        lines.append(f"<th>{label}</th>")
    lines.extend(["</tr>", "</thead>", "<tbody>"])

    best_worst: dict[str, tuple[float | None, float | None]] = {}
    for key, rule in metric_rules.items():
        values = [_metric_value(row[key]) for row in rows]
        values = [value for value in values if value is not None]
        if not values:
            best_worst[key] = (None, None)
            continue
        if rule == "high":
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
                if numeric is not None and best is not None and abs(numeric - best) < 1e-9:
                    classes.append("best")
                if numeric is not None and worst is not None and abs(numeric - worst) < 1e-9:
                    classes.append("worst")
            class_attr = f" class=\"{' '.join(classes)}\"" if classes else ""
            lines.append(f"<td{class_attr}>{value}</td>")
        lines.append("</tr>")
    lines.extend(["</tbody>", "</table>", "</div>", "</section>"])
    return lines


def _write_html(summary_rows: list[dict[str, str]], scenario_rows: list[dict[str, str]]) -> None:
    aggregate_columns = [
        ("method", "Method"),
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
    ]
    aggregate_rules = {
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
    scenario_columns = [
        ("method", "Method"),
        ("success", "Succ"),
        ("goal_reached_once", "Reach Once"),
        ("path_length", "Path"),
        ("time_to_goal_seconds", "T_goal (s)"),
        ("final_goal_distance", "Final Err"),
        ("min_clearance", "Min Clr"),
        ("collision", "Coll"),
        ("safety_violation", "Safe Viol"),
        ("path_efficiency", "Eff"),
    ]
    scenario_rules = {
        "success": "high",
        "goal_reached_once": "high",
        "path_length": "low",
        "time_to_goal_seconds": "low",
        "final_goal_distance": "low",
        "min_clearance": "high",
        "collision": "low",
        "safety_violation": "low",
        "path_efficiency": "high",
    }

    overall_rows: list[dict[str, str]] = []
    grouped_methods: dict[str, list[dict[str, str]]] = {}
    for row in summary_rows:
        grouped_methods.setdefault(row["method"], []).append(row)
    for method, rows in sorted(grouped_methods.items()):
        overall_rows.append(
            {
                "method": method,
                "success_rate": _format_number(_mean([float(r["success_rate"]) for r in rows]), 3),
                "goal_reached_once_rate": _format_number(_mean([float(r["goal_reached_once_rate"]) for r in rows]), 3),
                "mean_final_goal_distance": _format_number(_mean([float(r["mean_final_goal_distance"]) for r in rows]), 3),
                "collision_rate": _format_number(_mean([float(r["collision_rate"]) for r in rows]), 3),
                "safety_violation_rate": _format_number(_mean([float(r["safety_violation_rate"]) for r in rows]), 3),
                "mean_path_efficiency": _format_number(_mean([float(r["mean_path_efficiency"]) for r in rows]), 3),
            }
        )

    best_success = max(overall_rows, key=lambda row: float(row["success_rate"]))
    worst_success = min(overall_rows, key=lambda row: float(row["success_rate"]))
    best_reach_once = max(overall_rows, key=lambda row: float(row["goal_reached_once_rate"]))
    best_error = min(overall_rows, key=lambda row: float(row["mean_final_goal_distance"]))
    best_collision = min(overall_rows, key=lambda row: float(row["collision_rate"]))
    worst_collision = max(overall_rows, key=lambda row: float(row["collision_rate"]))

    strongest_model = max(
        summary_rows,
        key=lambda row: (float(row["success_rate"]), -float(row["collision_rate"]), -float(row["mean_final_goal_distance"])),
    )
    weakest_model = min(
        summary_rows,
        key=lambda row: (float(row["success_rate"]), -float(row["collision_rate"]), -float(row["mean_final_goal_distance"])),
    )

    lines = [
        "<!doctype html>",
        "<html lang=\"en\">",
        "<head>",
        "<meta charset=\"utf-8\">",
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">",
        "<title>Paper 2022 Metric Summary</title>",
        "<style>",
        ":root { color-scheme: dark; --bg: #111318; --panel: #191c22; --panel2: #20242c; --text: #eef2f6; --muted: #adb7c4; --grid: #303743; --best: #1f6f43; --worst: #7f2d2d; --accent: #7cc6ff; }",
        "* { box-sizing: border-box; }",
        "body { margin: 0; padding: 24px; background: linear-gradient(180deg, #0f1116, #161a21); color: var(--text); font: 15px/1.45 -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; }",
        "main { max-width: 1600px; margin: 0 auto; }",
        "h1, h2, h3 { margin: 0 0 12px; line-height: 1.15; }",
        "h1 { font-size: 32px; }",
        "h2 { margin-top: 28px; font-size: 22px; color: var(--accent); }",
        "h3 { margin-top: 18px; font-size: 18px; }",
        "p, li { color: var(--muted); }",
        ".intro-grid { display: grid; grid-template-columns: 1.1fr 0.9fr; gap: 18px; margin: 18px 0 28px; }",
        ".card { border: 1px solid var(--grid); border-radius: 16px; background: rgba(255,255,255,0.03); padding: 18px 18px 14px; box-shadow: 0 10px 30px rgba(0,0,0,0.12); }",
        ".card h3 { margin-top: 0; }",
        ".metric-list { margin: 0; padding-left: 18px; }",
        ".metric-list li { margin-bottom: 8px; }",
        ".legend { display: flex; gap: 12px; flex-wrap: wrap; margin: 14px 0 20px; }",
        ".chip { display: inline-flex; align-items: center; gap: 8px; border: 1px solid var(--grid); background: var(--panel); padding: 8px 12px; border-radius: 999px; }",
        ".swatch { width: 14px; height: 14px; border-radius: 4px; display: inline-block; }",
        ".best-swatch { background: var(--best); }",
        ".worst-swatch { background: var(--worst); }",
        ".table-section { margin-bottom: 22px; }",
        ".table-wrap { overflow-x: auto; border: 1px solid var(--grid); border-radius: 16px; background: rgba(255,255,255,0.02); box-shadow: 0 10px 30px rgba(0,0,0,0.18); }",
        "table { width: 100%; border-collapse: collapse; min-width: 980px; }",
        "th, td { padding: 11px 12px; border-bottom: 1px solid var(--grid); border-right: 1px solid var(--grid); text-align: right; white-space: nowrap; }",
        "th:first-child, td:first-child { text-align: left; }",
        "th { position: sticky; top: 0; background: var(--panel2); color: var(--text); font-weight: 700; z-index: 1; }",
        "tbody tr:nth-child(odd) td { background: rgba(255,255,255,0.01); }",
        "tbody tr:hover td { background: rgba(124,198,255,0.07); }",
        "td.best { background: color-mix(in srgb, var(--best) 70%, transparent) !important; font-weight: 700; }",
        "td.worst { background: color-mix(in srgb, var(--worst) 70%, transparent) !important; font-weight: 700; }",
        ".notes { margin: 0 0 8px; }",
        ".subtle { color: var(--muted); font-size: 14px; }",
        "@media (max-width: 980px) { .intro-grid { grid-template-columns: 1fr; } }",
        "</style>",
        "</head>",
        "<body>",
        "<main>",
        "<h1>Paper 2022 Metric Summary</h1>",
        "<p class=\"notes\">Friendlier benchmark summary across all current models and navigators. Green marks the best value and red marks the worst value within each table block.</p>",
        f"<p class=\"subtle\">Common benchmark step size: dt = {DEFAULT_DT:.2f} s.</p>",
        "<div class=\"intro-grid\">",
        "<section class=\"card\">",
        "<h3>Metric Glossary</h3>",
        "<ul class=\"metric-list\">",
        "<li><strong>Succ</strong>: number of successful runs in that table block.</li>",
        "<li><strong>Succ Rate</strong>: fraction of runs that finish inside the goal tolerance while also satisfying the safety criterion.</li>",
        "<li><strong>Reach Once</strong>: fraction of runs that enter the goal region at least once, even if they do not stay there at the final state.</li>",
        "<li><strong>Path</strong>: traveled trajectory length.</li>",
        "<li><strong>T_goal (s)</strong>: time to first reach the goal region, averaged only over runs with finite goal-reaching time.</li>",
        "<li><strong>Final Err</strong>: final distance to the goal, used here as an accuracy proxy.</li>",
        "<li><strong>Mean Clr</strong>: average closest-obstacle clearance summary for the block.</li>",
        "<li><strong>Worst Clr</strong>: smallest clearance observed in the block. Negative means penetration or collision.</li>",
        "<li><strong>Coll Rate</strong>: fraction of runs whose minimum clearance goes below zero.</li>",
        "<li><strong>Safe Viol</strong>: fraction of runs whose minimum clearance drops below the configured safety clearance margin.</li>",
        "<li><strong>Eff</strong>: path efficiency, computed as straight-line displacement divided by traveled path length.</li>",
        "</ul>",
        "</section>",
        "<section class=\"card\">",
        "<h3>How To Read Colors</h3>",
        "<ul class=\"metric-list\">",
        "<li>Green marks the best value within the current table block.</li>",
        "<li>Red marks the worst value within the current table block.</li>",
        "<li>Aggregate tables compare methods within the same model family.</li>",
        "<li>Per-scenario tables compare methods within the same scenario.</li>",
        "<li>For success, clearance, and efficiency, larger is better.</li>",
        "<li>For time, final error, collision rate, and safety violation rate, smaller is better.</li>",
        "<li>Cells with <strong>-</strong> mean the metric is undefined for that row, usually because the method never reached the goal.</li>",
        "</ul>",
        "</section>",
        "</div>",
        "<div class=\"legend\">",
        "<span class=\"chip\"><span class=\"swatch best-swatch\"></span> Best in table block</span>",
        "<span class=\"chip\"><span class=\"swatch worst-swatch\"></span> Worst in table block</span>",
        "</div>",
        "<h2>Aggregate By Model</h2>",
    ]

    for model in sorted({row["model"] for row in summary_rows}):
        model_rows = [row for row in summary_rows if row["model"] == model]
        lines.extend(_render_table(model.replace("_", " ").title(), model_rows, aggregate_columns, aggregate_rules))

    lines.append("<h2>Per-Scenario Breakdown</h2>")
    grouped_keys = sorted({(row["model"], row["scenario"]) for row in scenario_rows})
    for model, scenario in grouped_keys:
        rows = [row for row in scenario_rows if row["model"] == model and row["scenario"] == scenario]
        title = f"{model.replace('_', ' ').title()} - {scenario.replace('_', ' ')}"
        lines.extend(_render_table(title, rows, scenario_columns, scenario_rules))

    lines.extend(
        [
            "<h2>Overall Summary</h2>",
            "<section class=\"card\">",
            "<p><strong>Across all current benchmarks, the strongest method in general is "
            f"<code>{best_success['method']}</code></strong>, with the highest average success rate "
            f"({best_success['success_rate']}) across the model families now implemented.</p>",
            "<p><strong>The weakest method in general is "
            f"<code>{worst_success['method']}</code></strong>, with the lowest average success rate "
            f"({worst_success['success_rate']}).</p>",
            "<p>The most reliable \"almost-there\" behavior comes from "
            f"<code>{best_reach_once['method']}</code>, which has the best average goal-region reach-once rate "
            f"({best_reach_once['goal_reached_once_rate']}). "
            f"The best average final accuracy comes from <code>{best_error['method']}</code> "
            f"with mean final error {best_error['mean_final_goal_distance']}.</p>",
            "<p>From a safety perspective, the best average collision rate is achieved by "
            f"<code>{best_collision['method']}</code> ({best_collision['collision_rate']}), "
            f"while the worst average collision rate belongs to <code>{worst_collision['method']}</code> "
            f"({worst_collision['collision_rate']}).</p>",
            "<p>At the model-specific level, the single strongest benchmark block right now is "
            f"<code>{strongest_model['model']} / {strongest_model['method']}</code> "
            f"with success rate {strongest_model['success_rate']}. "
            f"The weakest block is <code>{weakest_model['model']} / {weakest_model['method']}</code> "
            f"with success rate {weakest_model['success_rate']}.</p>",
            "<p>A comparison caveat is important here: the current <code>Haddadin</code> baseline is not purely local in the same sense as MFI. "
            "In this clean repo it uses local obstacle distance and direction, but it also uses obstacle center or centroid information to build its transverse avoidance term. "
            "<code>MFI</code>, by contrast, relies only on local obstacle sensing, closest-point averaging, and goal relaxation. "
            "So when MFI performs comparably well, or better in some cases, it is doing so under a weaker information assumption and with a more reactive formulation.</p>",
            "<p>In practical terms, the current repo is strongest for the point-mass double-integrator benchmarks, "
            "especially the 3D paper-style variants. The main remaining weakness is the quadrotor benchmark family, "
            "where all methods still lag behind the point-mass results and need more controller tuning or model-specific navigation adaptation.</p>",
            "</section>",
        ]
    )

    lines.extend(["</main>", "</body>", "</html>"])
    OUTPUT_HTML.write_text("\n".join(lines) + "\n")


def main() -> None:
    rows = _load_rows()
    summary = _summarize(rows)
    per_scenario = _scenario_rows(rows)
    _write_csv(summary)
    _write_markdown(summary)
    _write_html(summary, per_scenario)
    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_MD}")
    print(f"Wrote {OUTPUT_HTML}")


if __name__ == "__main__":
    main()
