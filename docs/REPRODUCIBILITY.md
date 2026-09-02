# Reproducibility guide

## Scope

The code in this repository makes the planning concepts inspectable and testable. It does not reproduce the complete ROS robot or the historical physical experiments.

## Requirements

- Python 3.10 or newer.
- No third-party runtime dependencies for the planner tests.

## Commands

```bash
python -m unittest discover -s tests -v
```

The tests check that both A* modes reach the goal, simplified paths remain collision-free, invalid inputs are rejected, and the local planner returns a collision-free candidate. They validate the reconstructed code only and must not be interpreted as a reproduction of the historical experiment metrics.

## ROS recovery checklist

To reproduce the original project, recover the robot's `catkin_ws/src` directory and record:

1. Ubuntu and ROS versions.
2. Sensor drivers and frame IDs.
3. Complete TF tree.
4. SLAM and AMCL configuration.
5. Global and local planner plugins.
6. Costmap layers, footprints, and inflation radius.
7. Chassis protocol and controller gains.
8. One-command launch files.
9. Raw bags and evaluation scripts.
