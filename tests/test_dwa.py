from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from follow_me_planner.dwa import DWAConfig, RobotState, dwa_control


class DWATests(unittest.TestCase):
    def test_returns_collision_free_candidate(self):
        config = DWAConfig(predict_time=1.0, dt=0.1)
        state = RobotState(0.0, 0.0, 0.0)
        velocity, yaw_rate, trajectory = dwa_control(
            state,
            goal=(2.0, 0.0),
            obstacles=[(1.0, 0.0)],
            global_path=[(0.0, 0.0), (2.0, 0.0)],
            config=config,
        )
        self.assertGreaterEqual(velocity, 0.0)
        self.assertTrue(trajectory)
        for sample in trajectory:
            self.assertGreater(((sample.x - 1.0) ** 2 + sample.y**2) ** 0.5, config.robot_radius)


if __name__ == "__main__":
    unittest.main()
