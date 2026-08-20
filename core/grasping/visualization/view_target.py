"""Interactive 3D view of a saved target: its point cloud, every candidate
grasp, and the real gripper mesh stepped through with a slider.

This is the offline counterpart to docker/graspgenx/run_vis.sh -- it reads
only what run_pipeline.py already wrote under config/targets/<name>/, so it
needs neither a GraspGenX install nor a GPU. Output is a self-contained HTML
file; open it in any browser, no server involved.

Usage: python3 core/grasping/view_target.py [name ...] [--rotate none|identity|z90|z-90|z180]

With no arguments every target under config/targets/ is rendered.
"""

import sys
from pathlib import Path

# Runnable directly (python3 core/grasping/visualization/<this>.py), which puts
# only this directory on the path -- add core/ so `grasping` resolves.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import plotly.graph_objects as go
import trimesh
import yaml

from grasping.grasp_io import load_grasps
from grasping.gripper_frame import GRIPPERS_DIR, palm_from_canonical

TARGETS_DIR = Path(__file__).resolve().parents[3] / "config" / "targets"

AXIS_LENGTH = 0.04  # meters, per drawn grasp frame
AXES = [("x", "red"), ("y", "green"), ("z", "blue")]
MAX_POINTS = 30_000  # subsample above this, purely for browser responsiveness

# From hsrb_analytic_ik's HSRC parameters: a grasp is reachable only if the
# wrist origin -- the palm pulled back along its own approach axis -- lands
# inside this height band. Cheap enough to show here, and it explains why
# some high grasps never plan.
WRIST_OFFSET, WRIST_Z_MIN, WRIST_Z_MAX = 0.155, 0.0484, 1.3850


def load_target(directory):
    """Read cloud.ply and grasps.yaml into (points, poses, scores, meta)."""
    poses, scores, meta = load_grasps(directory / "grasps.yaml")
    points = np.asarray(trimesh.load(directory / "cloud.ply").vertices)
    return points, poses, scores, meta


def _gripper_mesh(gripper, rotate=None):
    """The gripper's visual mesh, put into hand_palm_link -- the frame the
    saved poses now use.

    vis_mesh.obj ships in GraspGenX's canonical frame (+Z approach, +X
    closing, body behind the origin), so it needs palm_from_canonical to be
    drawn at a hand_palm_link pose. `rotate` overrides that, and exists only
    to eyeball the alternatives; leave it None.
    """
    mesh = trimesh.load(GRIPPERS_DIR / gripper / "vis_mesh.obj", force="mesh")
    mesh.apply_transform(palm_from_canonical(gripper) if rotate is None else rotate)
    return mesh


def _rotation(name):
    """Named overrides for --rotate, all quarter turns about the approach
    axis. None means the correct palm_from_canonical."""
    if name in (None, "none"):
        return None
    if name == "identity":
        return np.eye(4)
    degrees = {"z90": 90.0, "z-90": -90.0, "z180": 180.0}[name]
    angle = np.radians(degrees)
    matrix = np.eye(4)
    matrix[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    return matrix


def _cloud_trace(points):
    if len(points) > MAX_POINTS:
        points = points[np.random.choice(len(points), MAX_POINTS, replace=False)]
    return go.Scatter3d(
        x=points[:, 0], y=points[:, 1], z=points[:, 2], mode="markers",
        marker=dict(size=1.5, color="lightslategray", opacity=0.55),
        name="point cloud", hoverinfo="skip")


def _axis_traces(poses):
    """Three line traces, one per axis colour. Each packs every grasp's
    segment into a single trace separated by None, so the browser draws three
    objects instead of 3N."""
    traces = []
    for axis, (name, color) in enumerate(AXES):
        xs, ys, zs = [], [], []
        for pose in poses:
            origin = pose[:3, 3]
            tip = origin + pose[:3, axis] * AXIS_LENGTH
            xs += [origin[0], tip[0], None]
            ys += [origin[1], tip[1], None]
            zs += [origin[2], tip[2], None]
        traces.append(go.Scatter3d(
            x=xs, y=ys, z=zs, mode="lines", line=dict(color=color, width=4),
            name=f"grasp {name}-axis", hoverinfo="skip", showlegend=False))
    return traces


def _wrist_z(poses):
    """Height of each grasp's wrist origin: the palm pulled back along its
    own +Z. The third column of the rotation is that approach axis."""
    return poses[:, 2, 3] - WRIST_OFFSET * poses[:, 2, 2]


def _origins_trace(poses, scores):
    wrist_z = _wrist_z(poses)
    reachable = (wrist_z >= WRIST_Z_MIN) & (wrist_z <= WRIST_Z_MAX)
    return go.Scatter3d(
        x=poses[:, 0, 3], y=poses[:, 1, 3], z=poses[:, 2, 3], mode="markers",
        marker=dict(
            size=7, color=scores, colorscale="Viridis", cmin=0, cmax=1,
            # A hollow circle marks a grasp whose wrist lands outside the
            # arm's height band: unreachable from any base position at all.
            symbol=["circle" if ok else "circle-open" for ok in reachable],
            line=dict(color="black", width=1),
            colorbar=dict(title="score", thickness=14, len=0.6)),
        name="grasps",
        customdata=np.stack([np.arange(len(scores)), scores, wrist_z, reachable], axis=1),
        hovertemplate=("grasp %{customdata[0]:.0f}<br>score %{customdata[1]:.2f}"
                       "<br>wrist z %{customdata[2]:.3f} m"
                       "<br>in reach band %{customdata[3]:.0f}<extra></extra>"))


def _gripper_traces(mesh, poses, scores):
    """One mesh per grasp, all hidden but the best -- the slider toggles
    which is visible."""
    best = int(np.argmax(scores))
    traces = []
    for i, pose in enumerate(poses):
        vertices = mesh.vertices @ pose[:3, :3].T + pose[:3, 3]
        traces.append(go.Mesh3d(
            x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
            i=mesh.faces[:, 0], j=mesh.faces[:, 1], k=mesh.faces[:, 2],
            color="crimson", opacity=0.45, flatshading=True,
            name=f"gripper @ grasp {i}", visible=(i == best), hoverinfo="skip"))
    return traces, best


def _slider(first_gripper_index, count, scores, wrist_z):
    """Steps that show exactly one gripper mesh at a time, leaving every
    trace before `first_gripper_index` untouched."""
    steps = []
    for i in range(count):
        visible = [None] * first_gripper_index + [j == i for j in range(count)]
        steps.append(dict(
            method="restyle",
            args=[{"visible": [v for v in visible if v is not None]},
                  list(range(first_gripper_index, first_gripper_index + count))],
            label=f"{i}"))
    return [dict(
        active=int(np.argmax(scores)), pad=dict(t=50), steps=steps,
        currentvalue=dict(prefix="gripper at grasp "))]


def build_figure(points, poses, scores, meta, rotate=None):
    traces = [_cloud_trace(points), *_axis_traces(poses), _origins_trace(poses, scores)]
    mesh = _gripper_mesh(meta["gripper"], _rotation(rotate))
    gripper_traces, best = _gripper_traces(mesh, poses, scores)
    first_gripper_index = len(traces)
    traces += gripper_traces

    wrist_z = _wrist_z(poses)
    unreachable = int(((wrist_z < WRIST_Z_MIN) | (wrist_z > WRIST_Z_MAX)).sum())
    subtitle = f"{len(poses)} grasps, best {scores.max():.2f}"
    if unreachable:
        subtitle += f" -- {unreachable} outside the arm's height band (hollow markers)"

    figure = go.Figure(traces)
    figure.update_layout(
        title=f"{meta['object_id']}  ({meta['gripper']}, frame {meta['frame_id']})<br>"
              f"<sub>{subtitle}</sub>",
        # aspectmode='data' keeps one metre the same length on all three axes,
        # without which grasp orientations read as skewed.
        scene=dict(xaxis_title="x [m]", yaxis_title="y [m]", zaxis_title="z [m]",
                   aspectmode="data"),
        sliders=_slider(first_gripper_index, len(poses), scores, wrist_z),
        margin=dict(l=0, r=0, t=70, b=0), height=800)
    return figure


def render(directory, rotate=None):
    points, poses, scores, meta = load_target(directory)

    # cloud.ply and grasps.yaml are written in the same breath by
    # run_pipeline.py, so a big gap between them means one of the two is left
    # over from an older run and the view is meaningless.
    drift = np.linalg.norm(points.mean(axis=0) - poses[:, :3, 3].mean(axis=0))
    if drift > 0.5:
        print(f"  WARNING: {directory.name}: cloud and grasps are {drift:.2f} m apart -- "
              f"stale files, re-run the pipeline for this target")

    output = directory / "view.html"
    # Inline rather than CDN: the dev container is often offline, and a view
    # that needs the network is a view that fails when you want it.
    figure = build_figure(points, poses, scores, meta, rotate)
    figure.write_html(output, include_plotlyjs="inline")
    print(f"{meta['object_id']}: {len(poses)} grasps, {len(points)} points -> {output}")


def main(argv):
    rotate = None
    if "--rotate" in argv:
        index = argv.index("--rotate")
        rotate = argv[index + 1]
        argv = argv[:index] + argv[index + 2:]

    directories = [TARGETS_DIR / n for n in argv] if argv else sorted(
        d for d in TARGETS_DIR.iterdir() if (d / "grasps.yaml").exists())
    if not directories:
        print(f"no targets found under {TARGETS_DIR}")
        return 1
    for directory in directories:
        render(directory, rotate)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
