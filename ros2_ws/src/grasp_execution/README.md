# grasp_execution

Sits between `core/run_pipeline.py`, which writes grasps, and the robot, which
executes them.

## 1. What is in this package

| file | what it does |
|---|---|
| [`launch/move_to_grasp.launch.py`](launch/move_to_grasp.launch.py) | reads a `grasps.yaml` and hands the poses to MoveIt as pose goals |
| [`launch/ik_solver.launch.py`](launch/ik_solver.launch.py) | starts Toyota's collision aware IK solver |
| `src/move_to_grasp.cpp` | the MoveIt client that the first launch file starts |

`core/placement/base_placement.py` is the Python client for the IK solver.

## 2. How to run it

```bash
# execute the newest saved grasp
ros2 launch grasp_execution move_to_grasp.launch.py

# or pick one
ros2 launch grasp_execution move_to_grasp.launch.py target:=pringles_can
ros2 launch grasp_execution move_to_grasp.launch.py grasp_file:=/path/to/grasps.yaml

# start the IK solver, then call /ik_solver_node/solve_ik_with_collision
ros2 launch grasp_execution ik_solver.launch.py
```

## 3. Why we launch the IK solver ourselves

`hsrb_ik_solver_node` ships its own `example.launch.py`. It hardcodes the
HSR-**B** plugin and you cannot override it from the command line. The B and C
arm geometries really do differ, so our launch file picks the C plugin instead.

Everything else is copied from Toyota's example, including the `convolution`
values.

* Toyota's version: `hsrb_manipulation/hsrb_ik_solver_node/example/example.launch.py:82-84`
* Ours: [`launch/ik_solver.launch.py:43-52`](launch/ik_solver.launch.py)

## 4. What the solver actually does

`SolveIkWithCollision` does not solve IK once. It searches a grid of base
positions. All of this lives in
`tmc_manipulation/tmc_ik_solver_node/src/ik_solver_plugin.cpp`.

**Step 1. Bound the search** (line 97).
`CalculateBaseCandidateMapSize` uses hsrb_analytic_ik's closed form to get a
ring of base positions from which the arm could reach the hand goal. The inner
radius is the arm fully retracted. The outer radius is the arm fully extended.
If the pose falls outside that ring, the call returns empty almost immediately.
That is what makes this a cheap check to run before asking MoveIt to plan.

**Step 2. Rasterise the ring into a grid** (lines 109-113).
The step size is `ik_base_resolution`. For each cell:

1. skip it if it is outside the ring
2. skip it if `IsInObstacle(x, y, obstacle_map)` says so (line 121). This is
   where the costmap that `base_placement.py` passes in gets used.
3. solve IK at that base position (line 127). There may be several arm
   configurations.
4. throw away any solution that collides (line 138)
5. keep the rest in `ik_results_map[x_i][y_i]`

**Step 3. Score how robust each solution is** (line 148).
This is the convolution. See `grid_distance_threhsold` below.

**Step 4. Keep only the winners** (line 151).
`ExtractIkReulsts` is called with `threshold = max_robustness`, and it does
`if (score < threshold) continue`. The threshold *is* the highest score found.

So the service returns only the solutions tied for best. It is not a ranked
list of everything reachable. An empty result means unreachable. A non empty
result is already a tight cluster.

## 5. Parameter trace

Every parameter we set, and where it is read.

| parameter | we set | default | read at | documented at |
|---|---|---|---|---|
| `ik_plugin_type` | `HsrcIkSolverPluginRobustToBasePositionError` | none, fatal if unset | `ik_solver_node.cpp:69` | `PKGDOC.rst:26` |
| `map_convolution_type` | `EuclideanDistanceMapConvolution` | none, fatal if unset | `ik_solver_plugin.cpp:74` | `PKGDOC.rst:30` |
| `convolution.ik_base_resolution` | `0.05` | `0.05` | `ik_solver_plugin.cpp:82` | `PKGDOC.rst:33` |
| `convolution.grid_distance_threhsold` | `1.5` | `1.0` | `map_convolution.cpp:109` | `PKGDOC.rst:35` |
| `convolution.ik_result_joint_distance_threshold` | not set | `1.0` | `map_convolution.cpp:39` | `PKGDOC.rst:34` |

Paths are relative to `ros2_ws/src/`. The two `.cpp` files are under
`tmc_manipulation/tmc_ik_solver_node/src/`. `PKGDOC.rst` is under
`tmc_manipulation/tmc_ik_solver_node/`, and it is written in Japanese.

Note that `grid_distance_threhsold` has a different default depending on the
convolution type. The Manhattan version reads it as an int with default `1`
(`map_convolution.cpp:85`). The Euclidean version, which is the one we use,
reads it as a double with default `1.0` (`map_convolution.cpp:109`).

## 6. `ik_base_resolution`, 0.05 m

The spacing of the base position grid from step 2.

It does three things:

1. **Sets the precision of the answer.** Base poses come back on a 5 cm grid.
2. **Sets the cost.** The number of IK and collision checks scales as
   `area / resolution²`. Halving it to 0.025 makes the solver do four times the
   work. This is the main compute knob, and it is why the whole call stays well
   under a millisecond.
3. **Gives the other parameter its unit.** The convolution stencil is measured
   in cells, so its real size is
   `grid_distance_threhsold × ik_base_resolution`.

## 7. `grid_distance_threhsold`, 1.5 cells

The radius, in cells, of the neighbourhood used to score robustness. Built at
`map_convolution.cpp:115-127`. A neighbour at offset `(x, y)` is kept if
`sqrt(x² + y²) <= threshold`, and it contributes weight `1 / distance`.

```
threshold = 1.5  (0.075 m)      threshold = 1.0  (0.05 m)

      x  x  x                         .  x  .
      x  ·  x                         x  ·  x
      x  x  x                         .  x  .

   8 neighbours                    4 neighbours
   max score 6.83                  max score 4.00
   (4 × 1.0 + 4 × 0.707)           (4 × 1.0)
```

The scoring loop is `MapConvolutionBase::Run` at `map_convolution.cpp:56-80`.
For each IK solution in a cell it visits every neighbour cell. If that
neighbour holds any solution close enough in joint space, it adds that
neighbour's weight.

So the score is a weighted count of nearby base cells where the arm could hold
roughly the same configuration.

A high score means the pose sits in the middle of a patch where the arm works.
A low score means it sits on an edge, where moving 5 cm changes the arm
configuration or kills the solution. That is what "RobustToBasePositionError"
in the plugin name refers to. It is robustness to parking error, not to sensing
error.

## 8. `ik_result_joint_distance_threshold`, default 1.0

This is what "close enough in joint space" means in step 7. We do not set it,
so the default applies.

Read at `map_convolution.cpp:39`, then squared straight away on line 40. It is
compared against the sum of squared joint differences plus the squared base yaw
difference. So two solutions count as the same arm configuration when they are
within about 1.0 rad of total joint space distance.

## 9. Do not fix the spelling of `grid_distance_threhsold`

The typo is Toyota's, not ours. It is the actual parameter key their C++ reads:

```cpp
// map_convolution.cpp:109
const auto distance_threshold = tmc_utils::GetParameter(
    node, "convolution.grid_distance_threhsold", 1.0);
```

The same misspelling appears at `map_convolution.cpp:85`, in both
`RCLCPP_FATAL` strings (lines 88 and 112), in `PKGDOC.rst:35`, in all six of
their unit tests in `test/test_map_convolution.cpp`, and in
`hsrb_ik_solver_node/example/example.launch.py:84`, which is where our value of
1.5 came from.

**Correcting it fails silently.** Here is `GetParameter`, from
`tmc_common/tmc_utils/include/tmc_utils/parameters.hpp:41-47`:

```cpp
if (node->has_parameter(name)) {
  return node->get_parameter(name).template get_value<ParameterType>();
} else {
  return node->template declare_parameter<ParameterType>(name, default_value);
}
```

A correctly spelled key is simply never found, so the default `1.0` is used
instead of our `1.5`.

You would expect the guard below it to catch that. It does not. The guard reads
`if (distance_threshold < 1.0)` and `1.0 < 1.0` is false, so it passes, even
though the fatal message says "must be greater than 1".

There is no error and no warning. The only effect is that the robustness
neighbourhood quietly drops from 8 cells to 4, losing the diagonals. The
robustness check then covers a smaller area and is no longer the same in every
direction, with nothing in the logs to say why.

## 10. Two things to watch out for

### Border cells are never scored

`SetNeighbors` computes `map_reducing_` as the largest `|x|` or `|y|` in the
stencil (`map_convolution.cpp:47-50`). At threshold 1.5 that is 1. Then `Run`
loops like this (`map_convolution.cpp:59`):

```cpp
for (auto x_i = map_reducing_; x_i < ik_results_map.size() - map_reducing_; ++x_i)
```

So one ring of cells at the edge of the reachable ring gets skipped. Those
cells cannot have a full neighbourhood. Push the threshold past 2.0 and you
lose two rings instead of one.

Base positions right at the limit of the arm's reach are therefore never
returned. That is by design, not a bug, but it is easy to be surprised by.

### The robustness radius is much smaller than nav2's goal tolerance

`1.5 cells × 0.05 m = 7.5 cm`.

But `hsrb_rosnav/hsrb_rosnav_config/config/nav2_params.yaml:124` sets
`xy_goal_tolerance: 0.25`. Nav2 will park the base up to 25 cm from where you
asked.

So the solver certifies that a pose tolerates about 7.5 cm of parking error,
while navigation can leave you 25 cm off. The certificate does not cover the
error we actually get.

If this turns out to matter, the options are:

1. tighten `xy_goal_tolerance` for grasp approaches only. Cheapest.
2. set `grid_distance_threhsold` to 5.0. Covers 25 cm, but that is about 80
   neighbours per cell and five lost border rings.
3. set `ik_base_resolution` to 0.1 and `grid_distance_threhsold` to 2.5. Covers
   the same 25 cm far more cheaply, at the cost of coarser output.

Measure before changing anything. The tied for best filtering in step 4 may
already return a cluster tight enough that its centre is fine. That centre is
what `base_placement.py` prints as `centred (x, y)`.

## 11. Source map

All paths relative to `ros2_ws/src/`.

| what | where |
|---|---|
| grid rasterisation, obstacle test, collision filter | `tmc_manipulation/tmc_ik_solver_node/src/ik_solver_plugin.cpp:95-152` |
| `IsInObstacle` | `ik_solver_plugin.cpp:34` |
| tied for best extraction | `ik_solver_plugin.cpp:48-67` |
| robustness convolution loop | `tmc_manipulation/tmc_ik_solver_node/src/map_convolution.cpp:56-80` |
| stencil construction, Euclidean | `map_convolution.cpp:106-130` |
| stencil construction, Manhattan | `map_convolution.cpp:82-104` |
| `GetParameter` fallback | `tmc_common/tmc_utils/include/tmc_utils/parameters.hpp:41-47` |
| parameter list, Japanese | `tmc_manipulation/tmc_ik_solver_node/PKGDOC.rst:26-35` |
| Toyota's example launch | `hsrb_manipulation/hsrb_ik_solver_node/example/example.launch.py:82-84` |
| nav2 goal tolerance | `hsrb_rosnav/hsrb_rosnav_config/config/nav2_params.yaml:124` |
