from __future__ import annotations

from typing import Protocol

import numpy as np


class Obstacle(Protocol):
    def closest_vector(self, position: np.ndarray) -> np.ndarray:
        ...

    def clearance(self, position: np.ndarray) -> float:
        ...

    def ray_intersection_distance(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        max_range: float,
    ) -> float | None:
        ...
