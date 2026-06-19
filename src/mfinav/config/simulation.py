from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class SimulationConfig:
    dt: float = 0.02
    steps: int = 6000
    kp_goal: float = 0.04
    kd_goal: float = 0.5
    kd_speed: float = 0.5
    kp_goal_relaxed: float = 0.04
    kp_geom: float = 5.0
    speed_limit: float = 1.5
    magni_bound: float = 2.5
    r_l: float = 4.0
    r_la: float = 2.5
    c_field: float = 12.0
    c_perp: float = 35.0
    delta_r: float = 0.5
    epsilon_current: float = 3e-6
    sensing_samples_per_obstacle: int = 15
    sensing_angular_window: float = 1.0
    goal_relaxation: bool = True
    use_legacy_goal_relaxation: bool = False
    max_acceleration: float = 4.0
    max_speed_norm: float = 2.0
    max_linear_speed: float = 1.0
    max_angular_speed: float = 2.5
    speed_gain: float = 0.35
    heading_gain: float = 2.5
    min_forward_factor: float = 0.2
    safety_clearance: float = 0.05
    goal_mode: str = "hybrid"
    field_mode: str = "pragmatic"
    sensing_mode: str = "analytic"
    sensor_range: float = 6.0
    goal_relaxation_mode: str = "legacy"


def make_paper_pd_config() -> SimulationConfig:
    return SimulationConfig(
        kp_goal=0.08,
        kd_goal=0.5,
        kd_speed=0.5,
        kp_goal_relaxed=0.08,
        kp_geom=5.0,
        speed_limit=1.5,
        magni_bound=2.5,
        r_l=3.0,
        r_la=2.0,
        c_field=10.0,
        c_perp=20.0,
        delta_r=0.5,
        epsilon_current=3e-6,
        goal_relaxation=True,
        use_legacy_goal_relaxation=False,
        goal_mode="pd",
        field_mode="paper",
        sensing_mode="raycast",
        sensor_range=6.0,
        goal_relaxation_mode="paper",
        max_acceleration=math.inf,
        max_speed_norm=math.inf,
    )


def make_paper_geometric_config() -> SimulationConfig:
    cfg = make_paper_pd_config()
    cfg.goal_mode = "geometric"
    return cfg


def make_paper_pd_3d_config() -> SimulationConfig:
    cfg = make_paper_pd_config()
    cfg.sensing_mode = "analytic"
    cfg.r_l = 4.0
    cfg.r_la = 2.0
    cfg.c_field = 15.0
    cfg.c_perp = 20.0
    cfg.speed_limit = 1.0
    cfg.kp_goal = 0.08
    cfg.kp_goal_relaxed = 0.08
    return cfg


def make_paper_geometric_3d_config() -> SimulationConfig:
    cfg = make_paper_pd_3d_config()
    cfg.goal_mode = "geometric"
    cfg.speed_limit = 0.5
    cfg.kp_goal = 0.12
    cfg.kp_goal_relaxed = 0.06
    cfg.kd_goal = 0.5
    cfg.kp_geom = 0.3
    cfg.c_field = 22.0
    cfg.c_perp = 20.0
    cfg.r_l = 4.0
    cfg.r_la = 2.0
    cfg.max_acceleration = 4.0
    cfg.max_speed_norm = 2.0
    return cfg


def make_paper_faithful_config() -> SimulationConfig:
    return make_paper_pd_config()


def make_pragmatic_config() -> SimulationConfig:
    return SimulationConfig(
        goal_mode="hybrid",
        field_mode="pragmatic",
    )
