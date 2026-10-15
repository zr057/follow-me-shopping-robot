"""Render a reproducible A* planning figure with no external dependencies."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from follow_me_planner.astar import astar_plan


def build_demo():
    grid = [[0] * 26 for _ in range(18)]
    for row in range(1, 16):
        if row != 7:
            grid[row][9] = 1
    for col in range(9, 23):
        if col != 18:
            grid[10][col] = 1
    for row in range(3, 8):
        grid[row][18] = 1
    start, goal = (15, 2), (2, 23)
    result = astar_plan(grid, start, goal, improved=True)
    assert result.reached_goal and all(grid[row][col] == 0 for row, col in result.raw_path)
    return grid, start, goal, result


def render(output: Path) -> None:
    grid, start, goal, result = build_demo()
    cell, left, top = 30, 30, 94
    width, height = len(grid[0]) * cell + left * 2, len(grid) * cell + 172

    def center(point):
        row, col = point
        return left + col * cell + cell / 2, top + row * cell + cell / 2

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f7fafc"/>',
        '<text x="30" y="40" font-family="Arial, sans-serif" font-size="24" font-weight="700" fill="#10243e">Reproducible A* planning demo</text>',
        '<text x="30" y="68" font-family="Arial, sans-serif" font-size="14" fill="#526477">Generated from the public reference implementation — not a historical RViz screenshot</text>',
    ]
    for row, values in enumerate(grid):
        for col, occupied in enumerate(values):
            fill = "#334a60" if occupied else "#ffffff"
            svg.append(f'<rect x="{left + col * cell}" y="{top + row * cell}" width="{cell}" height="{cell}" fill="{fill}" stroke="#d9e2ea"/>')

    raw_points = " ".join(f"{x:.1f},{y:.1f}" for x, y in map(center, result.raw_path))
    path_points = " ".join(f"{x:.1f},{y:.1f}" for x, y in map(center, result.path))
    svg.extend([
        f'<polyline points="{raw_points}" fill="none" stroke="#8fb8d8" stroke-width="6" stroke-linecap="round" stroke-linejoin="round" opacity="0.65"/>',
        f'<polyline points="{path_points}" fill="none" stroke="#0873b9" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>',
    ])
    for label, point, color in (("S", start, "#2e9b62"), ("G", goal, "#e07e2d")):
        x, y = center(point)
        svg.append(f'<circle cx="{x}" cy="{y}" r="11" fill="{color}" stroke="#ffffff" stroke-width="3"/>')
        svg.append(f'<text x="{x}" y="{y + 5}" text-anchor="middle" font-family="Arial, sans-serif" font-size="13" font-weight="700" fill="#ffffff">{label}</text>')

    metrics = f"Expanded nodes: {result.expanded_nodes}   Path length: {result.path_length:.1f} cells   Simplified turns: {result.turn_count}"
    svg.extend([
        f'<text x="30" y="{height - 38}" font-family="Arial, sans-serif" font-size="15" fill="#334a60">{metrics}</text>',
        f'<text x="{width - 260}" y="{height - 38}" font-family="Arial, sans-serif" font-size="13" fill="#526477">Dark cells: obstacles · Blue: planned path</text>',
        '</svg>',
    ])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(svg), encoding="utf-8")


if __name__ == "__main__":
    render(ROOT / "assets" / "simulation" / "astar-planning-demo.svg")
