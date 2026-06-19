from __future__ import annotations

from typing import Protocol

import numpy as np

from ..models.double_integrator import DoubleIntegratorState
from ..obstacles.base import Obstacle


class Navigator(Protocol):
    def command(
        self,
        state: DoubleIntegratorState,
        goal: np.ndarray,
        obstacle: Obstacle,
    ) -> np.ndarray:
        ...
