import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from follow_me_planner.astar import astar_plan


class AStarTests(unittest.TestCase):
    def test_baseline_and_improved_reach_goal(self):
        grid = [[0] * 12 for _ in range(12)]
        for row in range(2, 10):
            grid[row][6] = 1
        grid[7][6] = 0
        start, goal = (1, 1), (10, 10)
        baseline = astar_plan(grid, start, goal)
        improved = astar_plan(grid, start, goal, improved=True)
        self.assertTrue(baseline.reached_goal)
        self.assertTrue(improved.reached_goal)
        self.assertEqual(improved.path[0], start)
        self.assertEqual(improved.path[-1], goal)

    def test_improved_path_does_not_cross_obstacle(self):
        grid = [[0] * 10 for _ in range(10)]
        for col in range(2, 8):
            grid[5][col] = 1
        result = astar_plan(grid, (1, 1), (8, 8), improved=True)
        self.assertTrue(result.reached_goal)
        self.assertTrue(math.isfinite(result.path_length))
        for row, col in result.raw_path:
            self.assertEqual(grid[row][col], 0)

    def test_invalid_start_is_rejected(self):
        grid = [[0] * 4 for _ in range(4)]
        grid[0][0] = 1
        with self.assertRaises(ValueError):
            astar_plan(grid, (0, 0), (3, 3))


if __name__ == "__main__":
    unittest.main()
