from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config.simulation import SimulationConfig
from ..utils.math2d import EPS, _norm


@dataclass
class QuadrotorState:
    position: np.ndarray
    velocity: np.ndarray
    rotation: np.ndarray
    angular_velocity: np.ndarray
    thrust_total: float = 0.0
    thrust_rate: float = 0.0


def _clip_norm(vec: np.ndarray, limit: float) -> np.ndarray:
    magnitude = _norm(vec)
    if magnitude <= limit or magnitude < EPS:
        return vec
    return vec * (limit / magnitude)


def _skew(vec: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [0.0, -vec[2], vec[1]],
            [vec[2], 0.0, -vec[0]],
            [-vec[1], vec[0], 0.0],
        ],
        dtype=float,
    )


def _reorthonormalize(rotation: np.ndarray) -> np.ndarray:
    u, _, vh = np.linalg.svd(rotation)
    corrected = u @ vh
    if np.linalg.det(corrected) < 0.0:
        u[:, -1] *= -1.0
        corrected = u @ vh
    return corrected


class QuadrotorModel:
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config
        self.e3 = np.array([0.0, 0.0, 1.0], dtype=float)
        self.gravity = np.array([0.0, 0.0, -config.quadrotor_mass * config.quadrotor_gravity], dtype=float)
        self.inertia = np.diag(np.asarray(config.quadrotor_inertia, dtype=float))
        self.inv_inertia = np.diag(1.0 / np.asarray(config.quadrotor_inertia, dtype=float))
        self.allocation = np.array(
            [
                [config.quadrotor_thrust_coeff] * 4,
                [0.0, config.quadrotor_arm_length * config.quadrotor_thrust_coeff, 0.0, -config.quadrotor_arm_length * config.quadrotor_thrust_coeff],
                [-config.quadrotor_arm_length * config.quadrotor_thrust_coeff, 0.0, config.quadrotor_arm_length * config.quadrotor_thrust_coeff, 0.0],
                [config.quadrotor_drag_coeff, -config.quadrotor_drag_coeff, config.quadrotor_drag_coeff, -config.quadrotor_drag_coeff],
            ],
            dtype=float,
        )
        self.inv_allocation = np.linalg.inv(self.allocation)

    def clip_guidance(self, guidance: np.ndarray) -> np.ndarray:
        return _clip_norm(np.asarray(guidance, dtype=float), self.config.quadrotor_guidance_limit)

    def _euler_zyx(self, rotation: np.ndarray) -> tuple[float, float, float]:
        pitch = float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))
        roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
        yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
        return roll, pitch, yaw

    def step(self, state: QuadrotorState, guidance: np.ndarray, dt: float) -> tuple[QuadrotorState, dict[str, np.ndarray | float]]:
        guidance = self.clip_guidance(guidance)
        force_des = guidance + self.config.quadrotor_mass * self.config.quadrotor_gravity * self.e3
        thrust_total = float(np.clip(_norm(force_des), 0.0, self.config.quadrotor_max_total_thrust))

        desired_roll = float(
            np.clip(
                -self.config.quadrotor_force_to_tilt_gain * force_des[1] / max(thrust_total, EPS),
                -self.config.quadrotor_max_tilt,
                self.config.quadrotor_max_tilt,
            )
        )
        desired_pitch = float(
            np.clip(
                self.config.quadrotor_force_to_tilt_gain * force_des[0] / max(thrust_total, EPS),
                -self.config.quadrotor_max_tilt,
                self.config.quadrotor_max_tilt,
            )
        )
        roll, pitch, yaw = self._euler_zyx(state.rotation)
        desired_yaw = 0.0

        torque = np.array(
            [
                -self.config.quadrotor_angle_gain * (roll - desired_roll) - self.config.quadrotor_angle_damping * state.angular_velocity[0],
                -self.config.quadrotor_angle_gain * (pitch - desired_pitch) - self.config.quadrotor_angle_damping * state.angular_velocity[1],
                -self.config.quadrotor_yaw_gain * (yaw - desired_yaw) - self.config.quadrotor_yaw_damping * state.angular_velocity[2],
            ],
            dtype=float,
        )

        thrust_rate = float(np.clip((thrust_total - state.thrust_total) / max(dt, EPS), -self.config.quadrotor_max_thrust_rate, self.config.quadrotor_max_thrust_rate))
        thrust_torque = np.array([thrust_total, torque[0], torque[1], torque[2]], dtype=float)
        rotor_square = self.inv_allocation @ thrust_torque
        rotor_square = np.maximum(rotor_square, 0.0)
        rotor_speed = np.sqrt(rotor_square)
        rotor_speed = np.clip(rotor_speed, 0.0, self.config.quadrotor_max_rotor_speed)

        thrust_body = self.e3 * thrust_total
        acceleration = (state.rotation @ thrust_body + self.gravity) / self.config.quadrotor_mass
        velocity = state.velocity + acceleration * dt
        velocity = _clip_norm(velocity, self.config.max_speed_norm)
        position = state.position + velocity * dt

        omega_skew = _skew(state.angular_velocity)
        rotation_dot = state.rotation @ omega_skew
        rotation = _reorthonormalize(state.rotation + rotation_dot * dt)
        angular_acceleration = self.inv_inertia @ (self.inertia @ (omega_skew @ state.angular_velocity) + torque)
        angular_velocity = state.angular_velocity + angular_acceleration * dt
        angular_velocity = _clip_norm(angular_velocity, self.config.quadrotor_max_angular_speed)

        diagnostics: dict[str, np.ndarray | float] = {
            "guidance": guidance,
            "force_des": force_des,
            "torque": torque,
            "rotor_speed": rotor_speed,
            "acceleration": acceleration,
        }
        next_state = QuadrotorState(
            position=position,
            velocity=velocity,
            rotation=rotation,
            angular_velocity=angular_velocity,
            thrust_total=thrust_total,
            thrust_rate=thrust_rate,
        )
        return next_state, diagnostics
