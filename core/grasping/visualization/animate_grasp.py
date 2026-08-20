"""Animate the HSR hand approaching and closing on a saved target.

Plays a grasp the way it actually happens: the hand travels in along its own
approach axis with the fingers open, then the fingers close to the config's
`close` pose. Everything comes from files already in the repo -- the gripper
URDF, its link meshes, and config/targets/<name>/ -- so no GraspGenX install
and no GPU.

Usage: python3 core/grasping/animate_grasp.py <name> [--grasp N] [--standoff M]

Defaults to the highest-scoring grasp. Writes grasp_animation.html next to
the target, self-contained and openable offline.
"""

import json
import sys
import xml.etree.ElementTree as ElementTree
from pathlib import Path

# Runnable directly (python3 core/grasping/visualization/<this>.py), which puts
# only this directory on the path -- add core/ so `grasping` resolves.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import plotly.graph_objects as go
import trimesh
import yaml

from grasping.gripper_frame import GRIPPERS_DIR, palm_from_canonical
from grasping.visualization.view_target import TARGETS_DIR, load_target

APPROACH_FRAMES, CLOSE_FRAMES = 24, 16
DEFAULT_STANDOFF = 0.15  # meters back along the approach axis to start from


def _floats(text):
    return [float(v) for v in text.split()]


def _origin_matrix(element):
    """A URDF <origin> as a 4x4. rpy is fixed-axis XYZ, i.e. Rz @ Ry @ Rx."""
    matrix = np.eye(4)
    if element is None:
        return matrix
    roll, pitch, yaw = _floats(element.get("rpy", "0 0 0"))
    for angle, axis in ((roll, [1, 0, 0]), (pitch, [0, 1, 0]), (yaw, [0, 0, 1])):
        if angle:
            matrix = _rotation_about(np.array(axis, dtype=float), angle) @ matrix
    matrix[:3, 3] = _floats(element.get("xyz", "0 0 0"))
    return matrix


def load_gripper(gripper):
    """Parse gripper.urdf into (links, joints).

    links:  {name: (vertices in that link's own frame, faces)}
    joints: [(name, parent, child, origin 4x4, axis, mimic)] in declaration
            order, which is parent-before-child, so a single forward pass does
            FK. `mimic` is None or (source joint, multiplier, offset).
    """
    directory = GRIPPERS_DIR / gripper
    root = ElementTree.parse(directory / "gripper.urdf").getroot()

    links = {}
    for link in root.findall("link"):
        mesh_element = link.find("visual/geometry/mesh")
        if mesh_element is None:
            continue
        mesh = trimesh.load(directory / mesh_element.get("filename"), force="mesh")
        # A <visual><origin> offsets the mesh within its own link.
        visual = _origin_matrix(link.find("visual/origin"))
        vertices = np.asarray(mesh.vertices) @ visual[:3, :3].T + visual[:3, 3]
        links[link.get("name")] = (vertices, np.asarray(mesh.faces))

    joints = []
    for joint in root.findall("joint"):
        origin = _origin_matrix(joint.find("origin"))
        axis_element = joint.find("axis")
        axis = np.array(_floats(axis_element.get("xyz"))) if axis_element is not None else None
        element = joint.find("mimic")
        mimic = None if element is None else (
            element.get("joint"),
            float(element.get("multiplier", 1.0)),
            float(element.get("offset", 0.0)))
        joints.append((joint.get("name"), joint.find("parent").get("link"),
                       joint.find("child").get("link"), origin, axis, mimic))
    return links, joints


def _rotation_about(axis, angle):
    """Rodrigues, for a unit `axis`."""
    matrix = np.eye(4)
    cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    matrix[:3, :3] = (np.eye(3) + np.sin(angle) * cross
                      + (1 - np.cos(angle)) * cross @ cross)
    return matrix


def resolve_mimics(joints, values):
    """Joint angles with every mimic joint driven from its source.

    config.json stores a value for the mimic joints too, and those values
    contradict the URDF: it gives hand_l_distal_joint as +0.798 when the mimic
    says -1.0 * hand_motor_joint - 0.087 = -1.327. Trusting the stored numbers
    splays the fingers to 0.254 m when the real hand opens to 0.128 m. The
    URDF is the authority; the stored values for driven joints are ignored.
    """
    resolved = dict(values)
    for name, _, _, _, _, mimic in joints:
        if mimic:
            source, multiplier, offset = mimic
            # Reads `resolved`, not `values`, so a mimic of a mimic works as
            # long as the source is declared first -- which URDF ordering gives.
            resolved[name] = multiplier * resolved.get(source, 0.0) + offset
    return resolved


def forward_kinematics(joints, values):
    """{link: 4x4} in hand_palm_link coordinates, for the joint angles in
    `values` (missing joints are held at zero, mimics driven from source)."""
    values = resolve_mimics(joints, values)
    transforms = {"base_link": np.eye(4), "hand_palm_link": np.eye(4)}
    for name, parent, child, origin, axis, _ in joints:
        if parent not in transforms:
            continue
        local = origin
        if axis is not None:
            local = origin @ _rotation_about(axis, values.get(name, 0.0))
        transforms[child] = transforms[parent] @ local
    return transforms


def hand_vertices(links, joints, values):
    """Every visual link's vertices, posed and concatenated, plus the shared
    face list offset to match. One mesh per frame keeps the animation to a
    single trace."""
    transforms = forward_kinematics(joints, values)
    all_vertices, all_faces, offset = [], [], 0
    for name, (vertices, faces) in links.items():
        matrix = transforms[name]
        all_vertices.append(vertices @ matrix[:3, :3].T + matrix[:3, 3])
        all_faces.append(faces + offset)
        offset += len(vertices)
    return np.vstack(all_vertices), np.vstack(all_faces)


def grasp_sequence(gripper, standoff):
    """(joint values, retreat distance) per frame: approach with the hand
    open, then close in place."""
    config = json.loads((GRIPPERS_DIR / gripper / "config.json").read_text())
    open_pose, close_pose = config["open"], config["close"]

    frames = [(open_pose, standoff * (1.0 - i / (APPROACH_FRAMES - 1)))
              for i in range(APPROACH_FRAMES)]
    for i in range(CLOSE_FRAMES):
        t = i / (CLOSE_FRAMES - 1)
        frames.append(({k: (1 - t) * open_pose[k] + t * close_pose[k] for k in open_pose}, 0.0))
    return frames


def build_figure(points, pose, gripper, standoff):
    links, joints = load_gripper(gripper)
    to_palm = palm_from_canonical(gripper)
    sequence = grasp_sequence(gripper, standoff)

    def posed(values, retreat):
        vertices, faces = hand_vertices(links, joints, values)
        # Back off along the hand's own +Z, then into the world. to_palm is
        # identity for the URDF links -- they are already hand_palm_link --
        # but keeping it explicit documents which frame the vertices are in.
        world = pose @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -retreat], [0, 0, 0, 1]])
        return vertices @ world[:3, :3].T + world[:3, 3], faces

    first_vertices, faces = posed(*sequence[0])
    cloud = go.Scatter3d(
        x=points[:, 0], y=points[:, 1], z=points[:, 2], mode="markers",
        marker=dict(size=1.8, color="lightslategray", opacity=0.6),
        name="object", hoverinfo="skip")
    hand = go.Mesh3d(
        x=first_vertices[:, 0], y=first_vertices[:, 1], z=first_vertices[:, 2],
        i=faces[:, 0], j=faces[:, 1], k=faces[:, 2],
        color="crimson", opacity=0.75, flatshading=True, name="hand", hoverinfo="skip")

    frames = []
    for index, (values, retreat) in enumerate(sequence):
        vertices, _ = posed(values, retreat)
        frames.append(go.Frame(
            name=str(index),
            data=[go.Mesh3d(x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
                            i=faces[:, 0], j=faces[:, 1], k=faces[:, 2],
                            color="crimson", opacity=0.75, flatshading=True)],
            traces=[1]))

    figure = go.Figure(data=[cloud, hand], frames=frames)
    figure.update_layout(
        scene=dict(xaxis_title="x [m]", yaxis_title="y [m]", zaxis_title="z [m]",
                   aspectmode="data"),
        updatemenus=[dict(
            type="buttons", showactive=False, x=0.05, y=0.05,
            buttons=[
                dict(label="play", method="animate",
                     args=[None, dict(frame=dict(duration=45, redraw=True),
                                      fromcurrent=True, mode="immediate")]),
                dict(label="pause", method="animate",
                     args=[[None], dict(frame=dict(duration=0, redraw=False),
                                        mode="immediate")])])],
        sliders=[dict(
            steps=[dict(method="animate", label=str(i),
                        args=[[str(i)], dict(frame=dict(duration=0, redraw=True),
                                             mode="immediate")])
                   for i in range(len(sequence))],
            currentvalue=dict(prefix="frame "), pad=dict(t=45))],
        margin=dict(l=0, r=0, t=60, b=0), height=800)
    return figure


def main(argv):
    grasp_index, standoff = None, DEFAULT_STANDOFF
    for flag, cast in (("--grasp", int), ("--standoff", float)):
        if flag in argv:
            i = argv.index(flag)
            value = cast(argv[i + 1])
            argv = argv[:i] + argv[i + 2:]
            grasp_index, standoff = (value, standoff) if flag == "--grasp" else (grasp_index, value)

    if not argv:
        print(__doc__)
        return 1

    directory = TARGETS_DIR / argv[0]
    points, poses, scores, meta = load_target(directory)
    index = int(np.argmax(scores)) if grasp_index is None else grasp_index

    output = directory / "grasp_animation.html"
    figure = build_figure(points, poses[index], meta["gripper"], standoff)
    figure.update_layout(title=f"{meta['object_id']} -- grasp {index} "
                               f"(score {scores[index]:.2f}), {meta['gripper']}")
    figure.write_html(output, include_plotlyjs="inline")
    print(f"{meta['object_id']}: animating grasp {index} -> {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
