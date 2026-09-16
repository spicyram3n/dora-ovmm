"""Next-best-view by counting unseen voxels behind the target: active_grasp/nbv.py.

The policy fuses each depth image, casts rays from every candidate view through
the TSDF, and scores a view by how many voxels it would reveal on the far side
of the surface inside the target's box (ETH call these "rear side" voxels).
After each fusion active_grasp also predicts grasps on the volume (VGN), keeps
those closing inside the box, and stops early once the best grasp's quality has
stayed above a threshold for a window of views. The same branch is here with
the grasp network behind a `grasp_fn(points) -> (poses, qualities)` callable,
so GraspGenX (core/grasping/graspgenx_client.py) can take VGN's place.

numba is not installed here, so raycast() steps every ray of a view at once in
numpy instead of ETH's jit-compiled per-ray loop; the result is the same set of
voxels. Frames are (4, 4) matrices; `view` is base_from_camera, as ETH's `x`.
"""

import numpy as np

from .tsdf import UniformTSDFVolume, base_from_task


def raycast(
    voxel_size,
    tsdf_grid,
    ori,
    pos,
    fx,
    fy,
    cx,
    cy,
    u_min,
    u_max,
    v_min,
    v_max,
    t_min,
    t_max,
    t_step,
):
    """Voxels each pixel's ray visits before it first crosses a surface, (N, 3)."""
    # Make one camera ray for every sampled image pixel.
    u, v = np.meshgrid(np.arange(u_min, u_max), np.arange(v_min, v_max), indexing="ij")
    directions = np.stack(
        [(u.ravel() - cx) / fx, (v.ravel() - cy) / fy, np.ones(u.size)], 1
    )
    # Normalize each ray so its step distance is measured in metres.
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    # Rotate the rays from camera axes into the fusion cube axes.
    directions = directions @ ori.T
    resolution = tsdf_grid.shape[0]
    # Track which rays have not yet crossed a surface and their previous distance values.
    alive = np.ones(len(directions), dtype=bool)
    tsdf_prev = np.full(len(directions), -1.0)
    visited = []
    for t in np.arange(t_min, t_max, t_step):
        # Find the voxel reached by each ray at this distance.
        index = np.floor((pos + t * directions) / voxel_size).astype(np.int64)
        # Keep active rays whose current voxel lies within the cube.
        inside = np.flatnonzero(
            alive & np.all((index >= 0) & (index < resolution), axis=1)
        )
        i, j, k = index[inside].T
        tsdf = tsdf_grid[i, j, k]
        prev = tsdf_prev[inside]
        crossed = (tsdf * prev < 0) & (prev > -1)  # crossed a surface
        # Stop tracing a ray when its signed distance crosses a surface.
        alive[inside[crossed]] = False
        visited.append(index[inside[~crossed]])
        tsdf_prev[inside[~crossed]] = tsdf[~crossed]
    if not visited:
        return np.zeros((0, 3), dtype=np.int64)
    return np.concatenate(visited)


def information_gain(tsdf, bbox, task_from_base, intrinsic, view, downsample):
    """NextBestView.ig_fn: rear-side voxels inside `bbox` that `view` would see."""
    tsdf_grid, voxel_size = tsdf.get_grid(), tsdf.voxel_size
    tsdf_grid = -1.0 + 2.0 * tsdf_grid  # Open3D maps tsdf to [0,1]

    # Use fewer image pixels to make view scoring cheaper.
    fx = intrinsic[0, 0] / downsample
    fy = intrinsic[1, 1] / downsample
    cx = intrinsic[0, 2] / downsample
    cy = intrinsic[1, 2] / downsample

    # Project the target box into the image to limit which rays we trace.
    camera_from_base = np.linalg.inv(view)
    corners = np.array(
        [camera_from_base[:3, :3] @ p + camera_from_base[:3, 3] for p in bbox.corners]
    ).T
    u = (fx * corners[0] / corners[2] + cx).round().astype(int)
    v = (fy * corners[1] / corners[2] + cy).round().astype(int)
    u_min, u_max = u.min(), u.max()
    v_min, v_max = v.min(), v.max()

    t_min = 0.0
    t_max = corners[2].max()  # Camera-depth bound; oblique rays may need a longer range.
    t_step = np.sqrt(3) * voxel_size  # Step by one voxel diagonal.

    # Move the candidate camera into the fusion cube's coordinate frame.
    view = task_from_base @ view
    ori, pos = view[:3, :3], view[:3, 3]

    voxel_indices = raycast(
        voxel_size,
        tsdf_grid,
        ori,
        pos,
        fx,
        fy,
        cx,
        cy,
        u_min,
        u_max,
        v_min,
        v_max,
        t_min,
        t_max,
        t_step,
    )

    # Count each visited voxel once, keeping only those inside the target box.
    indices = np.unique(voxel_indices, axis=0)
    # Express the target bounds in voxel units for the inside-box check.
    bbox_min = (task_from_base[:3, :3] @ bbox.min + task_from_base[:3, 3]) / voxel_size
    bbox_max = (task_from_base[:3, :3] @ bbox.max + task_from_base[:3, 3]) / voxel_size
    mask = np.all((indices > bbox_min) & (indices < bbox_max), axis=1)
    i, j, k = indices[mask].T
    tsdfs = tsdf_grid[i, j, k]
    # Negative known distances identify voxels just behind the surface.
    return int(np.logical_and(tsdfs > -1.0, tsdfs < 0.0).sum())


class NextBestView:
    """Fuse target views, score new camera positions, and track reliable grasps."""

    def __init__(
        self,
        intrinsic,
        length=0.3,
        resolution=40,
        max_views=80,
        min_gain=10,
        downsample=20,
        window_size=12,
        grasp_fn=None,
        qual_thresh=0.9,
        finger_depth=0.05,
        reachable_fn=None,
        tip_margin=0.03,
    ):
        self.intrinsic = np.asarray(intrinsic, dtype=float)
        self.length, self.resolution = length, resolution
        self.max_views, self.min_gain, self.downsample = max_views, min_gain, downsample
        self.T = window_size
        self.grasp_fn, self.qual_thresh, self.finger_depth = (
            grasp_fn,
            qual_thresh,
            finger_depth,
        )
        self.reachable_fn = reachable_fn
        # Allow a small tip margin because the observed box may miss hidden surfaces.
        self.tip_margin = tip_margin

    def activate(self, bbox, view_sphere):
        self.bbox = bbox
        self.view_sphere = view_sphere
        self.base_from_task = base_from_task(bbox, self.length)
        self.task_from_base = np.linalg.inv(self.base_from_task)
        # Create an empty volume for the new target and clear the previous run state.
        self.tsdf = UniformTSDFVolume(self.length, self.resolution)
        self.views = []
        self.x_d = None
        self.done = False
        self.info = {}
        self.best_grasp = None
        # Keep a rolling window of grasp scores for each voxel.
        self.qual_hist = np.zeros((self.T,) + (self.resolution,) * 3, np.float32)

    def integrate(self, img, x):
        self.views.append(x)
        # Invert the camera pose because fusion expects cube points expressed in the camera.
        self.tsdf.integrate(img, self.intrinsic, np.linalg.inv(self.task_from_base @ x))
        if self.grasp_fn is not None:
            self.predict_grasps()

    def predict_grasps(self):
        """Predict grasps on the target surface and record the best scores by voxel."""
        points = (
            np.asarray(self.tsdf.get_scene_cloud().points) + self.base_from_task[:3, 3]
        )
        # Send only target points to the model so it does not grasp the table.
        margin = self.tsdf.voxel_size
        inside = np.all((points > self.bbox.min - margin) & (points < self.bbox.max + margin), axis=1)
        points = points[inside]
        self.best_grasp = None
        # Wait for enough target surface points before asking for grasp predictions.
        if len(points) < 50:
            return
        poses, qualities = self.grasp_fn(points)
        poses, qualities = self.filter_grasps(poses, qualities)
        # Reuse the oldest history slot for this view's grasp scores.
        t = (len(self.views) - 1) % self.T
        self.qual_hist[t] = 0.0
        if len(poses):
            i, j, k = self.voxel_of(poses[:, :3, 3])
            self.qual_hist[t, i, j, k] = np.maximum(
                self.qual_hist[t, i, j, k], qualities
            )
            # Remember the highest-scoring grasp from the current fused surface.
            best = int(np.argmax(qualities))
            self.best_grasp = (poses[best], float(qualities[best]))

    def filter_grasps(self, poses, qualities):
        """Keep high-quality grasps with tips inside the target box, then check reachability."""
        poses, qualities = (
            np.asarray(poses, dtype=float),
            np.asarray(qualities, dtype=float),
        )
        # Find each fingertip centre by moving along the grasp approach axis.
        tips = poses[:, :3, 3] + self.finger_depth * poses[:, :3, 2]
        low, high = (self.bbox.min - self.tip_margin, self.bbox.max + self.tip_margin)
        # Reject weak grasps and grasps whose fingertips miss the expanded target box.
        keep = (qualities > self.qual_thresh) & np.all((tips > low) & (tips < high), axis=1)
        poses, qualities = (poses[keep], qualities[keep])
        if self.reachable_fn is not None and len(poses):
            # Best first: the IK service is one call per grasp.
            order = np.argsort(-qualities)
            poses, qualities = (poses[order], qualities[order])
            reachable = np.asarray(self.reachable_fn(poses), dtype=bool)
            poses, qualities = (poses[reachable], qualities[reachable])
        return poses, qualities

    def voxel_of(self, points_base):
        # Convert positions into cube coordinates before finding voxel indices.
        task = points_base @ self.task_from_base[:3, :3].T + self.task_from_base[:3, 3]
        index = np.clip(
            (task / self.tsdf.voxel_size).astype(int), 0, self.resolution - 1
        )
        return index.T

    def best_grasp_prediction_is_stable(self):
        """Check for a score in every history slot and a mean score above 0.9."""
        if self.best_grasp:
            i, j, k = self.voxel_of(self.best_grasp[0][None, :3, 3])
            # Read this grasp voxel's scores across the whole recent-view window.
            qs = self.qual_hist[:, i[0], j[0], k[0]]
            if np.count_nonzero(qs) == self.T and np.mean(qs) > 0.9:
                return True
        return False

    def ig_fn(self, view):
        # Score this candidate view using the current fused volume and target box.
        return information_gain(
            self.tsdf,
            self.bbox,
            self.task_from_base,
            self.intrinsic,
            view,
            self.downsample,
        )

    def cost_fn(self, view):
        return 1.0

    def update(self, img, x):
        # Stop before taking another view if the budget or grasp-stability test is satisfied.
        if len(self.views) > self.max_views or self.best_grasp_prediction_is_stable():
            self.done, self.info, self.x_d = (True, {}, None)
            return
        self.integrate(img, x)
        views = self.view_sphere.candidates()
        # Estimate how much hidden target surface each feasible view could reveal.
        gains = np.array([self.ig_fn(v) for v in views], dtype=float)
        costs = np.array([self.cost_fn(v) for v in views], dtype=float)
        if not len(views) or gains.sum() == 0:
            self.done, self.info, self.x_d = (True, {}, None)
            return
        # Compare normalized information gain and motion cost for each view.
        utilities = gains / np.sum(gains) - costs / np.sum(costs)
        i = int(np.argmax(utilities))
        nbv, gain = views[i], gains[i]
        self.info = dict(views=views, gains=gains, utilities=utilities, best=i)
        # After the initial view window, stop if the best remaining gain is too small.
        if gain < self.min_gain and len(self.views) > self.T:
            self.done = True
        self.x_d = nbv
