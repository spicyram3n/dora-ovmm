"""Gazebo's ground truth as Instances, read straight from the .world file.

No simulator needs to be running: a world is a list of models with poses, and
each model's SDF gives the collision geometry those poses apply to. That keeps
building the scene graph an offline step.

Collision geometry rather than visual, because it is the one that is reliably
made of primitives; the few models that are a mesh either way go through
trimesh. Each piece contributes the eight corners of its box *in local
coordinates*, transformed afterwards, so a model placed at an angle stays at
an angle and relations.py's hull footprint keeps its meaning.
"""

import os
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from perception.pointcloud import transform_points
from scene_graph.instance import Instance

# Where hsrb_apartment_world.launch.py spawns the robot, with yaw 0. ROS's
# odom frame starts wherever the robot did, so world coordinates run this much
# larger than the odom ones the rest of the pipeline speaks
# (perception.camera_ros2.BASE_FRAME, and every Nav2 goal). Another world
# passes its own spawn.
APARTMENT_SPAWN = (5.0, 6.6)


def _pose(element):
    """<pose>x y z roll pitch yaw</pose> as a 4x4, identity when absent."""
    matrix = np.eye(4)
    text = element.findtext("pose")
    if not text:
        return matrix
    values = np.fromstring(text, sep=" ")
    matrix[:3, :3] = Rotation.from_euler("xyz", values[3:6]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def _corners(lower, upper):
    """The eight corners of a box, which is all the graph wants from any one
    piece of geometry."""
    return np.array([[x, y, z] for x in (lower[0], upper[0])
                     for y in (lower[1], upper[1]) for z in (lower[2], upper[2])])


def _roots(world_path):
    """Where model:// uris are looked up: the world's sibling models/, plus
    GAZEBO_MODEL_PATH, since a world may well name a model that does not ship
    beside it."""
    siblings = os.path.join(os.path.dirname(os.path.dirname(world_path)), "models")
    roots = [siblings] + os.environ.get("GAZEBO_MODEL_PATH", "").split(":")
    return [root for root in roots if root and os.path.isdir(root)]


def _resolve(uri, roots):
    """model://name/rest as a real path, or None if no root holds it."""
    for root in roots:
        path = os.path.join(root, uri.replace("model://", "", 1))
        if os.path.exists(path):
            return path
    return None


def _model_sdf(uri, roots):
    """A model's SDF path. model.config names the file, which is why this is
    not just model.sdf: these ship as model-1_4.sdf."""
    directory = _resolve(uri, roots)
    if directory is None:
        return None
    return os.path.join(directory, ET.parse(os.path.join(directory, "model.config"))
                        .getroot().findtext("sdf"))


def _collada_unit(path):
    """Metres per unit, as COLLADA declares in its own header.

    Some of these meshes are authored in decimetres, and Gazebo's loader
    applies that. trimesh reports it as the string '0.1 * meters' and then
    declines to convert it, and force="mesh" drops even the report, so it gets
    read here instead. Other mesh formats carry no unit and are metres by
    convention.
    """
    if not path.endswith(".dae"):
        return 1.0
    unit = ET.parse(path).getroot().find("{*}asset/{*}unit")
    return float(unit.get("meter", 1.0)) if unit is not None else 1.0


def _geometry_corners(geometry, roots):
    """Local-frame corners of one <geometry>, or None for a kind not modelled
    here (an SDF plane, say, which has no extent to speak of)."""
    box = geometry.find("box")
    if box is not None:
        extents = np.fromstring(box.findtext("size"), sep=" ")
        return _corners(-extents / 2, extents / 2)

    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius, length = float(cylinder.findtext("radius")), float(cylinder.findtext("length"))
        extents = np.array([2 * radius, 2 * radius, length])
        return _corners(-extents / 2, extents / 2)

    sphere = geometry.find("sphere")
    if sphere is not None:
        radius = float(sphere.findtext("radius"))
        return _corners(np.full(3, -radius), np.full(3, radius))

    mesh = geometry.find("mesh")
    if mesh is not None:
        path = _resolve(mesh.findtext("uri"), roots)
        if path is None:
            return None
        scale = np.fromstring(mesh.findtext("scale", "1 1 1"), sep=" ") * _collada_unit(path)
        bounds = trimesh.load(path, force="mesh").bounds * scale
        return _corners(bounds[0], bounds[1])
    return None


def _collision_points(model, pose, roots):
    """Every collision corner of a model, in the world frame."""
    points = []
    for link in model.iter("link"):
        link_pose = pose @ _pose(link)
        for collision in link.iter("collision"):
            corners = _geometry_corners(collision.find("geometry"), roots)
            if corners is not None:
                points.append(transform_points(link_pose @ _pose(collision), corners))
    return np.vstack(points) if points else np.empty((0, 3))


def _models(world, roots):
    """(pose source, model, label, name) for every model in the world.

    Two ways in: <include>, where the geometry lives in another file, and a
    top-level <model> written out inline, which is how megaweb2015.world
    carries its bookshelves. The inline case is its own pose source.
    """
    for element in world.findall("include"):
        uri = element.findtext("uri")
        sdf = _model_sdf(uri, roots)
        if sdf is None:
            # Never hit by apartment.world, but a world naming a model that is
            # not on GAZEBO_MODEL_PATH is an ordinary Gazebo mishap, and the
            # alternative is a parse error thrown from somewhere far away.
            print(f"  skipping {uri}: no model found under {roots}")
            continue
        label = uri.rsplit("/", 1)[-1]
        yield element, ET.parse(sdf).getroot().find("model"), label, element.findtext("name") or label

    for model in world.findall("model"):
        yield model, model, model.get("name"), model.get("name")


def load_world(path, origin=APARTMENT_SPAWN):
    """Every model in a .world file, as Instances in the robot's odom frame.

    The label is the model's *type* (its model:// name), since that is what
    the furniture vocabulary reads; the instance's own name is kept alongside
    for traceability. What is furniture and what is a movable object is left
    to that vocabulary rather than to the SDF's <static> flag, which marks the
    doors dynamic and would get them badly wrong.
    """
    roots = _roots(path)
    world = ET.parse(path).getroot().find("world")
    shift = np.eye(4)
    shift[:2, 3] = -np.asarray(origin, float)

    instances = []
    for element, model, label, name in _models(world, roots):
        points = _collision_points(model, shift @ _pose(element), roots)
        if len(points):
            instances.append(Instance(label, points, name=name))
    return instances
