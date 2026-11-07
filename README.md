# Two-robot assembly of a sheet-metal frame with xArm6

Technical assignment for ETH Zurich (drawing AMZ-SA-26-001) · Davide Giuseppe Raciti · November 2026

Two UFACTORY xArm6 robots must assemble four U-profiles with tolerances of ±0.05 mm, while the robot repeatability is ±0.1 mm. My solution decouples the accuracy from the robots: the robots only transport the parts into the lead-ins of a fixture, and the fixture centres, pushes and clamps them. The full reasoning is in the report.

## Contents

| Folder | What it contains |
| --- | --- |
| `report/` | The report: `Raciti_ETH_xArm6_assignment.pdf` |
| `reachability/` | `reach_check.py`: reachability check of the cell layout, based on the official xArm6 DH parameters |
| `control/sfc/` | SFC of the cell, drawn in Schneider SoMachine 4.3: `SFC_scheme.project` (open with SoMachine / EcoStruxure Machine Expert) and image |
| `control/petri_net/` | Petri net of the mutual exclusion on the fixture, modelled and analysed in PIPE: `petri_net_mutual_exclusion.xml` (PNML, open with PIPE), image and invariant analysis |
| `ros/xarm_cell_viz/` | ROS Noetic package: the two xArm6 (official UFACTORY model) in the cell layout, shown in RViz at the main poses |
| `figures/` | Figures of the report |

## Reachability check

For every pick and place point of the layout, the script solves the inverse kinematics with the gripper pointing down and computes the arm extension (shoulder-to-wrist distance divided by its maximum). All six points are reachable at 43–61 % extension, inside my design band of 30–75 %.

```
pip install -r reachability/requirements.txt
python reachability/reach_check.py
```

The DH model was cross-checked against UFACTORY's ROS kinematics file (`xarm_ros2`, `xarm6_default_kinematics.yaml`): less than 0.003 mm difference over 1000 random poses. The check covers geometry only (no collisions, no dynamics); the tool length of 200 mm is assumed.

## ROS visualization (ROS Noetic)

The package loads the official dual-arm model from UFACTORY's `xarm_ros` (`xarm_description`), places R1 at (0, −0.4) m and R2 at (0, +0.4) m, and shows the robots at the poses computed by `reach_check.py`, with the fixture and the trays as markers.

```
# in a catkin workspace that contains xarm_description (github.com/xArm-Developer/xarm_ros) and this package
catkin_make && source devel/setup.bash
roslaunch xarm_cell_viz cell.launch scene:=r1_place_far
```

Scenes: `home`, `pick`, `r1_place_near`, `r1_place_far`, `r2_place_left`, `r2_place_right`.

## Mutual exclusion

The PIPE invariant analysis gives the place invariant `R1 in fixture + R2 in fixture + fixture free = 1`, so the two robots are never in the fixture at the same time.

## Use of AI tools

AI tools were used for drafting support and code assistance; the concept, design decisions and verification are my own.
