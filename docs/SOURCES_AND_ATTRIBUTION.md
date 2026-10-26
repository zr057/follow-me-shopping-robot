# Sources and attribution

## Undergraduate project sources

The repository narrative is based on the archived project application, interim report, design/experiment report, and formal final report for **“面向智慧零售的多模态感知自主跟随购物车”**.

The project documents establish Rong Zhang's role, project scope, reported performance, limitations, and the historical development timeline.

The RViz mapping screenshot found in the proposal archive was not produced by the project vehicle and is therefore excluded from the public evidence.

## Authorized technical reference

The planning-method structure was informed by the locally stored Zhang Xiang thesis materials supplied by the user for this task. Relevant concepts include:

- Cartographer/AMCL organization for mapping and localization.
- Goal-oriented A* direction pruning.
- Obstacle-density-aware heuristic weighting.
- Two-stage redundant-node removal.
- Global A* plus DWA local planning.
- UWB-based dynamic-goal following architecture.

Three user-authorized photographs from the thesis folder document the confirmed final four-wheel platform, corridor layout, and static obstacle-avoidance test. No thesis diagram or paragraph is copied verbatim. The planning figure and Python implementation were created anew for transparency, and numerical results from the thesis are not presented as Rong Zhang's project results.

## Third-party archive

The archived `Shopping Robot of HFUT.rar` contains an older `ros_arduino_bridge` distribution, a CH341 serial driver, and motor documentation. These materials are not copied into this repository because their upstream licenses and compatibility need separate review.
