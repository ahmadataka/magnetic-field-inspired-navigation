from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from ..config.simulation import SimulationConfig


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class DifferentialDriveState:
    position: np.ndarray
    heading: float
    linear_speed: float = 0.0
    angular_speed: float = 0.0

    @property
    def velocity(self) -> np.ndarray:
        return self.linear_speed * np.array(
            [math.cos(self.heading), math.sin(self.heading)],
            dtype=float,
        )


@dataclass
class DifferentialDriveCommand:
    linear_speed: float
    angular_speed: float
    desired_heading: float
    heading_error: float


class DifferentialDriveModel:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def guidance_to_command(
        self,
        state: DifferentialDriveState,
        guidance: np.ndarray,
    ) -> DifferentialDriveCommand:
        guidance = np.asarray(guidance, dtype=float)
        magnitude = float(np.linalg.norm(guidance))
        if magnitude <= 1e-9:
            return DifferentialDriveCommand(
                linear_speed=0.0,
                angular_speed=0.0,
                desired_heading=state.heading,
                heading_error=0.0,
            )

        desired_heading = math.atan2(float(guidance[1]), float(guidance[0]))
        heading_error = _wrap_angle(desired_heading - state.heading)
        angular_speed = max(
            -self.config.max_angular_speed,
            min(self.config.max_angular_speed, self.config.heading_gain * heading_error),
        )

        if abs(heading_error) <= 0.5 * math.pi:
            alignment = max(self.config.min_forward_factor, math.cos(heading_error))
        else:
            alignment = 0.0
        linear_speed = min(
            self.config.max_linear_speed,
            self.config.speed_gain * magnitude,
        ) * alignment
        return DifferentialDriveCommand(
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            desired_heading=desired_heading,
            heading_error=heading_error,
        )

    def step(
        self,
        state: DifferentialDriveState,
        command: DifferentialDriveCommand,
        dt: float,
    ) -> DifferentialDriveState:
        heading_mid = state.heading + 0.5 * command.angular_speed * dt
        heading = _wrap_angle(state.heading + command.angular_speed * dt)
        velocity = command.linear_speed * np.array(
            [math.cos(heading_mid), math.sin(heading_mid)],
            dtype=float,
        )
        position = state.position + velocity * dt
        return DifferentialDriveState(
            position=position,
            heading=heading,
            linear_speed=command.linear_speed,
            angular_speed=command.angular_speed,
        )
