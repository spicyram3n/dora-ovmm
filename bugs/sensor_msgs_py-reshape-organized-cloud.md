# sensor_msgs_py: `reshape_organized_cloud` swaps rows and columns

**Status:** open upstream (present in Humble and in `rolling` as of 2026-09-11).
**Affects:** `sensor_msgs_py.point_cloud2.read_points` / `read_points_numpy`,
any caller passing `reshape_organized_cloud=True` on a non-square cloud.
**Found:** while writing `nbv/scripts/build_octomap.py`, 2026-09-11.

## Symptom

The octomap node produced a wall that was a quarter too short in one axis: the
points it integrated spanned y −0.245..0.500 where the camera had published
−0.495..0.500. No exception, no warning, just less of the scene than expected.
The same points fed through the same grid offline gave 368 occupied cells; over
ROS the node reported 288.

## Root cause

`/opt/ros/humble/lib/python3.10/site-packages/sensor_msgs_py/point_cloud2.py:128`

```python
if reshape_organized_cloud and cloud.height > 1:
    points = points.reshape(cloud.width, cloud.height)
```

A PointCloud2 is stored row-major: `height` rows of `width` points each. The
reshape therefore has to be `(height, width)`. Reshaping to `(width, height)`
keeps every point but lays them out transposed, so row and column indices mean
the wrong thing.

Reproduction, a 480x640 cloud in one process, no transport involved:

```python
pts = np.stack([x, y, z], -1)            # (480, 640, 3), x spans -0.5..0.5
msg = PointCloud2(height=480, width=640, point_step=12,
                  row_step=12*640, data=pts.tobytes())
back = read_points_numpy(msg, ["x","y","z"], reshape_organized_cloud=True)
back.shape                               # (640, 480, 3)  <- wrong way round
np.array_equal(back, pts)                # False
```

## Why it stayed silent

The total point count is unchanged and every point is still present, so
anything that flattens the array immediately is unaffected. It only bites code
that indexes the 2D structure — which is the only reason to pass the flag at
all. Ours decimated with `[::4, ::4]`, and on a transposed array that walks
across rows instead of down them, sampling an uneven quarter of the frame.
`height == width` would hide it completely.

## Fix

Do not use the flag. Reshape at the call site, where the intent is visible:

```python
points = read_points_numpy(msg, ["x", "y", "z"])
points = points.reshape(msg.height, msg.width, 3)[::STRIDE, ::STRIDE].reshape(-1, 3)
```

`build_octomap.py` no longer reads PointCloud2 at all — it deprojects the
depth image instead, which is a third of the bytes — so the workaround is not
in the tree. Keep it in mind for the next caller. Nothing in this workspace
passes `reshape_organized_cloud`:

```bash
grep -rn "reshape_organized_cloud" --include=*.py /home/ws
```

We deliberately did not patch `/opt/ros`: it is outside version control and is
replaced on a Dev Container rebuild.

## Verification

With the fix, `build_octomap.py` against a synthetic 480x640 wall reports 368
occupied cells, matching the offline computation on the same points exactly.
