"""Compact DWA-style local planner for a differential-drive robot."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

PointF = tuple[float, float]


@dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    yaw: float
    velocity: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class DWAConfig:
    max_speed: float = 0.8
    min_speed: float = 0.0
    max_yaw_rate: float = math.radians(60)
    max_accel: float = 0.5
    max_yaw_accel: float = math.radians(120)
    velocity_resolution: float = 0.05
    yaw_rate_resolution: float = math.radians(5)
    dt: float = 0.1
    predict_time: float = 2.0
    robot_radius: float = 0.30
    heading_weight: float = 0.35
    clearance_weight: float = 0.25
    speed_weight: float = 0.15
    global_path_weight: float = 0.25


def _sample(low: float, high: float, resolution: float) -> list[float]:
    if high < low:
        return []
    count = max(1, int(math.floor((high - low) / resolution)))
    values = [low + index * resolution for index in range(count + 1)]
    if values[-1] < high - 1e-9:
        values.append(high)
    return values


def _motion(state: RobotState, velocity: float, yaw_rate: float, dt: float) -> RobotState:
    yaw = state.yaw + yaw_rate * dt
    return RobotState(
        x=state.x + velocity * math.cos(yaw) * dt,
        y=state.y + velocity * math.sin(yaw) * dt,
        yaw=yaw,
        velocity=velocity,
        yaw_rate=yaw_rate,
    )


def _trajectory(state: RobotState, velocity: float, yaw_rate: float, config: DWAConfig) -> list[RobotState]:
    result = [state]
    elapsed = 0.0
    current = state
    while elapsed < config.predict_time - 1e-9:
        current = _motion(current, velocity, yaw_rate, config.dt)
        result.append(current)
        elapsed += config.dt
    return result


def _distance_to_points(state: RobotState, points: Sequence[PointF]) -> float:
    if not points:
        return math.inf
    return min(math.hypot(state.x - x, state.y - y) for x, y in points)


def _heading_error(state: RobotState, goal: PointF) -> float:
    desired = math.atan2(goal[1] - state.y, goal[0] - state.x)
    return abs(math.atan2(math.sin(desired - state.yaw), math.cos(desired - state.yaw)))


def dwa_control(
    state: RobotState,
    goal: PointF,
    obstacles: Sequence[PointF],
    global_path: Sequence[PointF],
    config: DWAConfig = DWAConfig(),
) -> tuple[float, float, list[RobotState]]:
    """Select velocity and yaw rate using a dynamic-window trajectory search."""
    velocity_low = max(config.min_speed, state.velocity - config.max_accel * config.dt)
    velocity_high = min(config.max_speed, state.velocity + config.max_accel * config.dt)
    yaw_low = max(-config.max_yaw_rate, state.yaw_rate - config.max_yaw_accel * config.dt)
    yaw_high = min(config.max_yaw_rate, state.yaw_rate + config.max_yaw_accel * config.dt)

    best_score = math.inf
    best = (0.0, 0.0, [state])
    for velocity in _sample(velocity_low, velocity_high, config.velocity_resolution):
        for yaw_rate in _sample(yaw_low, yaw_high, config.yaw_rate_resolution):
            trajectory = _trajectory(state, velocity, yaw_rate, config)
            clearance = min(_distance_to_points(sample, obstacles) for sample in trajectory)
            if clearance <= config.robot_radius:
                continue
            endpoint = trajectory[-1]
            heading = _heading_error(endpoint, goal) / math.pi
            clearance_cost = 1.0 / max(clearance - config.robot_radius, 1e-6)
            speed_cost = (config.max_speed - velocity) / max(config.max_speed, 1e-6)
            route_cost = _distance_to_points(endpoint, global_path)
            score = (
                config.heading_weight * heading
                + config.clearance_weight * clearance_cost
                + config.speed_weight * speed_cost
                + config.global_path_weight * route_cost
            )
            if score < best_score:
                best_score = score
                best = velocity, yaw_rate, trajectory
    return best
