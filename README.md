# FollowMe: Multimodal Autonomous Shopping Cart

**An indoor follow-me mobile robot integrating mapping, path planning, obstacle avoidance, and differential-drive control.**

[中文](README_CN.md) · [Project facts](docs/PROJECT_FACTS.md) · [Reproducibility](docs/REPRODUCIBILITY.md) · [Sources](docs/SOURCES_AND_ATTRIBUTION.md)

<p align="center">
  <img src="assets/prototype/final-prototype.png" width="760" alt="FollowMe shopping-cart prototype">
</p>

## Results at a glance

The project produced a physical shopping-cart prototype and tested indoor mapping, localization, path tracking, following, and obstacle avoidance. The values below are records from the final project report.

| Evaluation item | Reported result |
|---|---:|
| Mean following distance | 1.59 m |
| Path-tracking error | < 0.15 m |
| Repeat localization error — X | ±0.05 m |
| Repeat localization error — Y | ±0.10 m |
| Repeat heading error | ±5° |
| Dynamic obstacle avoidance | Demonstrated in an indoor corridor |

> The archived folder does not contain the raw trial logs or the number of obstacle-avoidance trials. For that reason, the reported 100% success rate is not used as a headline metric here.

## My contribution

**Role:** Project lead · Path-planning developer · System-integration coordinator

- Designed the grid-based A* global path-planning workflow.
- Implemented eight-neighbor search, obstacle constraints, path reconstruction, and path simplification.
- Participated in the integration of global planning and local obstacle avoidance.
- Integrated mapping/localization, planning, and differential-drive chassis control.
- Used closed-loop control to track planned paths and coordinated simulation and physical tests.
- Investigated localization drift, trajectory error, control stability, and module communication issues.

## Physical prototype and experiments

### From a manual cart to a mobile-robot prototype

| Base cart | Integrated prototype |
|---|---|
| <img src="assets/prototype/base-cart.jpg" width="420" alt="Original folding cart"> | <img src="assets/prototype/final-prototype.png" width="420" alt="Integrated mobile prototype"> |

The team converted a folding shopping cart into a differential-drive platform and integrated onboard computation and range sensing for indoor navigation.

### Indoor mapping

<p align="center">
  <img src="assets/mapping/rviz-map.png" width="620" alt="Indoor occupancy map displayed in RViz">
</p>

LiDAR measurements were used to build a two-dimensional occupancy map that supports localization and path planning.

### Corridor obstacle-avoidance test

<p align="center">
  <img src="assets/experiments/corridor-obstacle-test.jpg" width="480" alt="Corridor obstacle-avoidance experiment with traffic cones">
</p>

Traffic cones were placed in an indoor corridor to test planning, trajectory tracking, and obstacle avoidance on a physical mobile platform.

### Computing and sensing hardware

| Onboard controller | 2D LiDAR |
|---|---|
| <img src="assets/hardware/controller.jpg" width="420" alt="Onboard controller"> | <img src="assets/hardware/lidar.jpg" width="420" alt="2D LiDAR sensor"> |

## System design

<p align="center">
  <img src="assets/architecture/technical-route.png" width="920" alt="Project technical route">
</p>

The system connects four functional stages:

1. **Perception:** LiDAR, visual sensing, and odometry acquire environmental and robot-state information.
2. **Mapping and localization:** SLAM builds an occupancy map and estimates the cart pose.
3. **Planning:** global planning generates a route while local planning reacts to nearby obstacles.
4. **Control:** velocity commands are converted into differential wheel motion for path tracking.

The formal final report indicates that the laser-based pipeline was the primary implemented system; full visual target perception and multimodal fusion remained incomplete.

## Core planning work

### Global planning

The planner represents the environment as a two-dimensional grid and applies A* search to generate a collision-free route. The project work covered:

- eight-direction neighbor expansion;
- map-boundary and obstacle checks;
- heuristic-guided node selection;
- parent-based path reconstruction;
- removal of redundant turns and safe path shortcutting.

### Local obstacle avoidance

The local planner evaluates short candidate trajectories, rejects collision-prone motion, and balances goal heading, obstacle clearance, forward speed, and consistency with the global route.

### Trajectory control

Position and heading errors are converted into linear and angular velocity commands. Closed-loop control then adjusts the left and right wheel speeds to follow the planned route.

## Reference implementation

The repository includes a compact, testable reconstruction of the planning logic:

- [`astar.py`](src/follow_me_planner/astar.py): baseline and improved grid planning;
- [`dwa.py`](src/follow_me_planner/dwa.py): DWA-style local trajectory evaluation;
- [`tests/`](tests): path-safety and planner regression tests.

```bash
python -m unittest discover -s tests -v
```

This code was reconstructed from the archived project material and the authorized Zhang Xiang thesis reference. It is not presented as the original ROS workspace used for the historical physical experiments.

## Project information

| Item | Description |
|---|---|
| Project | Multimodal Perception Autonomous Follow-Me Shopping Cart for Smart Retail |
| Program | Undergraduate Innovation Training Project, Hefei University of Technology |
| Period | May 2025 – April 2026 |
| Role | Project lead |
| Main responsibility | Path planning, motion control, and system integration |

## Current limitations

- Visual target recognition was not fully integrated with the complete navigation pipeline.
- Tests were conducted mainly in campus indoor and corridor environments rather than a crowded supermarket.
- The original ROS packages, raw experiment logs, ROS bags, firmware, and complete demo video are not present in the archived folder.

No open-source license is granted yet. See [Sources and attribution](docs/SOURCES_AND_ATTRIBUTION.md) for the distinction between historical project evidence, reconstructed code, and reference material.
