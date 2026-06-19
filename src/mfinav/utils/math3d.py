from __future__ import annotations

import numpy as np

from .math2d import EPS, _norm, _unit


def _cross3(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)


def _orthogonal_projection(vec: np.ndarray, normal: np.ndarray) -> np.ndarray:
    normal_hat = _unit(normal)
    if _norm(normal_hat) < EPS:
        return vec.copy()
    return vec - float(np.dot(vec, normal_hat)) * normal_hat


__all__ = [
    "EPS",
    "_cross3",
    "_norm",
    "_orthogonal_projection",
    "_unit",
]
