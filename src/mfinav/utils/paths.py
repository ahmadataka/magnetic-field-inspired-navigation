from __future__ import annotations

from pathlib import Path


def _ensure(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def benchmark_artifact_dir(root: Path, suite: str) -> Path:
    return _ensure(root / "artifacts" / "benchmarks" / suite)


def smoke_artifact_dir(root: Path, name: str) -> Path:
    return _ensure(root / "artifacts" / "smoke_tests" / name)


def diagnostic_artifact_dir(root: Path, name: str) -> Path:
    return _ensure(root / "artifacts" / "diagnostics" / name)


def reference_artifact_dir(root: Path, name: str) -> Path:
    return _ensure(root / "artifacts" / "reference" / name)


def summary_artifact_dir(root: Path, name: str) -> Path:
    return _ensure(root / "artifacts" / "summaries" / name)


def generated_world_dir(root: Path, platform: str, suite: str) -> Path:
    return _ensure(root / "webots" / "worlds" / "generated" / platform / suite)
