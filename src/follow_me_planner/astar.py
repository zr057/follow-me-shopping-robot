"""Baseline and research-inspired grid A* planners.

The improved mode is a transparent reconstruction from archived research
descriptions. It combines goal-oriented direction pruning, obstacle-aware
heuristic weighting, and collision-checked path simplification.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from time import perf_counter
from typing import Iterable, Sequence

Grid = Sequence[Sequence[int]]
Point = tuple[int, int]

_MOVES: tuple[Point, ...] = (
    (-1, 0), (-1, 1), (0, 1), (1, 1),
    (1, 0), (1, -1), (0, -1), (-1, -1),
)


@dataclass(frozen=True)
class AStarResult:
    path: list[Point]
    raw_path: list[Point]
    expanded_nodes: int
    runtime_seconds: float
    path_length: float
    turn_count: int
    cumulative_turn_degrees: float
    reached_goal: bool


def _inside(grid: Grid, point: Point) -> bool:
    r, c = point
    return 0 <= r < len(grid) and 0 <= c < len(grid[0])


def _free(grid: Grid, point: Point) -> bool:
    return _inside(grid, point) and grid[point[0]][point[1]] == 0


def _euclidean(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _valid_move(grid: Grid, current: Point, move: Point) -> bool:
    nxt = current[0] + move[0], current[1] + move[1]
    if not _free(grid, nxt):
        return False
    if move[0] and move[1]:
        # Prevent diagonal corner cutting.
        side_a = current[0] + move[0], current[1]
        side_b = current[0], current[1] + move[1]
        return _free(grid, side_a) and _free(grid, side_b)
    return True


def _goal_oriented_moves(current: Point, goal: Point) -> tuple[Point, ...]:
    goal_angle = math.atan2(goal[0] - current[0], goal[1] - current[1])

    def angular_error(move: Point) -> float:
        angle = math.atan2(move[0], move[1])
        return abs(math.atan2(math.sin(angle - goal_angle), math.cos(angle - goal_angle)))

    return tuple(sorted(_MOVES, key=angular_error)[:5])


def _obstacle_prefix(grid: Grid) -> list[list[int]]:
    prefix = [[0] * (len(grid[0]) + 1) for _ in range(len(grid) + 1)]
    for row in range(len(grid)):
        row_total = 0
        for col in range(len(grid[0])):
            row_total += int(grid[row][col] != 0)
            prefix[row + 1][col + 1] = prefix[row][col + 1] + row_total
    return prefix


def _obstacle_ratio(prefix: Sequence[Sequence[int]], current: Point, goal: Point) -> float:
    r0, r1 = sorted((current[0], goal[0]))
    c0, c1 = sorted((current[1], goal[1]))
    occupied = (
        prefix[r1 + 1][c1 + 1]
        - prefix[r0][c1 + 1]
        - prefix[r1 + 1][c0]
        + prefix[r0][c0]
    )
    total = (r1 - r0 + 1) * (c1 - c0 + 1)
    return occupied / total


def _reconstruct(came_from: dict[Point, Point], goal: Point) -> list[Point]:
    path = [goal]
    while path[-1] in came_from:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


def _bresenham(a: Point, b: Point) -> Iterable[Point]:
    r0, c0 = a
    r1, c1 = b
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    while True:
        yield r0, c0
        if (r0, c0) == (r1, c1):
            break
        twice = 2 * err
        if twice > -dc:
            err -= dc
            r0 += sr
        if twice < dr:
            err += dr
            c0 += sc


def _has_clearance(grid: Grid, point: Point, clearance: int) -> bool:
    for dr in range(-clearance, clearance + 1):
        for dc in range(-clearance, clearance + 1):
            check = point[0] + dr, point[1] + dc
            if not _free(grid, check):
                return False
    return True


def _line_is_free(grid: Grid, a: Point, b: Point, clearance: int) -> bool:
    points = list(_bresenham(a, b))
    if not all(_has_clearance(grid, point, clearance) for point in points):
        return False
    for first, second in zip(points, points[1:]):
        dr, dc = second[0] - first[0], second[1] - first[1]
        if dr and dc:
            if not _has_clearance(grid, (first[0] + dr, first[1]), clearance):
                return False
            if not _has_clearance(grid, (first[0], first[1] + dc), clearance):
                return False
    return True


def _remove_collinear(path: list[Point]) -> list[Point]:
    if len(path) < 3:
        return path[:]
    compact = [path[0]]
    for index in range(1, len(path) - 1):
        prev = compact[-1]
        current = path[index]
        nxt = path[index + 1]
        first = current[0] - prev[0], current[1] - prev[1]
        second = nxt[0] - current[0], nxt[1] - current[1]
        if first[0] * second[1] != first[1] * second[0]:
            compact.append(current)
    compact.append(path[-1])
    return compact


def simplify_path(grid: Grid, path: list[Point], clearance: int = 0) -> list[Point]:
    """Remove collinear points, then shortcut segments with line-of-sight checks."""
    path = _remove_collinear(path)
    if len(path) < 3:
        return path
    simplified = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1:
            if _line_is_free(grid, path[anchor], path[candidate], clearance):
                break
            candidate -= 1
        simplified.append(path[candidate])
        anchor = candidate
    return simplified


def _path_metrics(path: Sequence[Point]) -> tuple[float, int, float]:
    if len(path) < 2:
        return 0.0, 0, 0.0
    headings = [
        math.atan2(b[0] - a[0], b[1] - a[1])
        for a, b in zip(path, path[1:])
    ]
    length = sum(_euclidean(a, b) for a, b in zip(path, path[1:]))
    changes = [
        abs(math.atan2(math.sin(b - a), math.cos(b - a)))
        for a, b in zip(headings, headings[1:])
    ]
    return length, sum(change > 1e-9 for change in changes), math.degrees(sum(changes))


def astar_plan(
    grid: Grid,
    start: Point,
    goal: Point,
    *,
    improved: bool = False,
    heuristic_gain: float = 0.8,
    shortcut_clearance: int = 0,
) -> AStarResult:
    """Plan a path on a binary occupancy grid.

    Cells with value 0 are traversable. Non-zero cells are obstacles. Improved
    mode intentionally uses a weighted heuristic, so it prioritizes efficiency
    and smoothness rather than guaranteeing the same optimality properties as
    conventional admissible A*.
    """
    if not grid or not grid[0]:
        raise ValueError("grid must be non-empty")
    if not _free(grid, start) or not _free(grid, goal):
        raise ValueError("start and goal must be free cells")

    started = perf_counter()
    frontier: list[tuple[float, int, Point]] = [(0.0, 0, start)]
    came_from: dict[Point, Point] = {}
    g_score: dict[Point, float] = {start: 0.0}
    closed: set[Point] = set()
    serial = 0
    obstacle_prefix = _obstacle_prefix(grid) if improved else None

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        closed.add(current)
        if current == goal:
            raw = _reconstruct(came_from, goal)
            path = simplify_path(grid, raw, shortcut_clearance) if improved else raw
            length, turns, cumulative = _path_metrics(path)
            return AStarResult(
                path=path,
                raw_path=raw,
                expanded_nodes=len(closed),
                runtime_seconds=perf_counter() - started,
                path_length=length,
                turn_count=turns,
                cumulative_turn_degrees=cumulative,
                reached_goal=True,
            )

        candidates = _goal_oriented_moves(current, goal) if improved else _MOVES
        valid_candidates = [move for move in candidates if _valid_move(grid, current, move)]
        # Preserve practical completeness when all preferred directions are blocked.
        if improved and not valid_candidates:
            valid_candidates = [move for move in _MOVES if _valid_move(grid, current, move)]

        ratio = _obstacle_ratio(obstacle_prefix, current, goal) if obstacle_prefix is not None else 1.0
        weight = 1.0 + heuristic_gain * (1.0 - ratio) if improved else 1.0

        for move in valid_candidates:
            nxt = current[0] + move[0], current[1] + move[1]
            tentative = g_score[current] + math.hypot(move[0], move[1])
            if tentative >= g_score.get(nxt, math.inf):
                continue
            came_from[nxt] = current
            g_score[nxt] = tentative
            serial += 1
            priority = tentative + weight * _euclidean(nxt, goal)
            heapq.heappush(frontier, (priority, serial, nxt))

    return AStarResult(
        path=[], raw_path=[], expanded_nodes=len(closed),
        runtime_seconds=perf_counter() - started, path_length=math.inf,
        turn_count=0, cumulative_turn_degrees=0.0, reached_goal=False,
    )
