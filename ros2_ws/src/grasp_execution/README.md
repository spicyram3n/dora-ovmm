# Grasp execution

Send saved grasp poses to MoveIt, or start Toyota's IK solver to find reachable base positions.

## Execute a saved grasp

Use a built, sourced ROS workspace with the robot and MoveIt running. In simulation, start MoveIt with:

```bash
ros2 launch hsrb_moveit_config demo.py description_package:=hsrc_description \
  description_file:=hsrc1s.urdf.xacro use_sim_time:=true
```

In another ROS terminal, choose **one**:

```bash
# Newest config/targets/*/grasps.yaml
ros2 launch grasp_execution move_to_grasp.launch.py

# A target folder
ros2 launch grasp_execution move_to_grasp.launch.py target:=pringles_can

# An explicit file (overrides target)
ros2 launch grasp_execution move_to_grasp.launch.py grasp_file:=/path/to/grasps.yaml
```

For hardware, pass `use_sim_time:=false`. Defaults are HSRC and MoveIt's `whole_body` group.

A saved `grasps.yaml` is required. The former `core/run_pipeline.py` generation command is absent from this checkout.

## Find a reachable base position

Start the solver in one ROS terminal:

```bash
ros2 launch grasp_execution ik_solver.launch.py
```

From the repository root in another terminal, with saved grasps in `odom`:

```bash
python3 core/navigation/base_placement.py pringles --costmap
```

Service: `/ik_solver_node/solve_ik_with_collision`.
The launch selects the **HSRC** plugin; Toyota's example hardcodes the different HSR-B geometry.

## IK settings

Edit [ik_solver.launch.py](launch/ik_solver.launch.py).

| Parameter | Value | Effect |
| --- | --- | --- |
| `convolution.ik_base_resolution` | `0.05` m | Base search grid spacing; halving it roughly quadruples grid work |
| `convolution.grid_distance_threhsold` | `1.5` cells | Scores solutions using nearby reachable base cells |
| `convolution.ik_result_joint_distance_threshold` | `1.0` (default) | Joint/base-yaw similarity used for scoring |

Keep the spelling **`threhsold`**. Toyota's code reads that exact key; correcting it silently uses the default.

The solver checks a base-position grid, filters collisions, scores nearby solutions, and returns only those tied for the best score. It does not return all reachable poses. Border cells without a full scoring neighborhood are skipped.

## Before relying on the result

- The scoring neighborhood is about **7.5 cm** (`1.5 × 0.05 m`), while Nav2 allows **25 cm** position error. Check reachability at the actual parked pose.
- If needed, measure and tune Nav2's approach tolerance or the IK grid/neighborhood. A larger neighborhood costs more and excludes more border cells.
- Collision checks depend on the supplied environment. Search currently leaves it empty, so arm–furniture collisions are not checked.
- A timeout is a service error, not an unreachable result. A reachable hand pose does not guarantee a stable grasp.

## Code map

| File | Purpose |
| --- | --- |
| [move_to_grasp.launch.py](launch/move_to_grasp.launch.py) | Select a grasp file and pass poses to MoveIt |
| [move_to_grasp.cpp](src/move_to_grasp.cpp) | Execute MoveIt pose goals |
| [base_placement.py](../../../core/navigation/base_placement.py) | Python IK client |

Toyota's solver implementation is under `ros2_ws/src/tmc_manipulation/tmc_ik_solver_node/`: `src/ik_solver_plugin.cpp` handles grid search and filtering; `src/map_convolution.cpp` handles scoring; `PKGDOC.rst` lists parameters.
