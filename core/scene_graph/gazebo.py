"""Gazebo's ground truth as Instances, read straight from the .world file."""

import os
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from core.perception.pointcloud import transform_points
from core.utils.geometry import box_corners
from .instance import LABELS, Instance, scannet_class


def _pose(element):
    """<pose>x y z roll pitch yaw</pose> as a 4x4, identity when absent."""
    matrix = np.eye(4)
    pose = element.find("pose")
    # Reject pose formats that this loader cannot resolve correctly.
    if pose is not None and (
        pose.get("relative_to")
        or pose.get("degrees") == "true"
        or pose.get("rotation_format", "euler_rpy") != "euler_rpy"
    ):
        raise ValueError(
            "This loader supports parent-relative xyz/rpy poses in radians only"
        )
    text = element.findtext("pose")
    if not text:
        return matrix
    values = np.fromstring(text, sep=" ")
    if len(values) != 6:
        raise ValueError("SDF pose must contain x y z roll pitch yaw")
    # Convert roll, pitch, and yaw to a rotation matrix and add XYZ translation.
    matrix[:3, :3] = Rotation.from_euler("xyz", values[3:6]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def _corners(lower, upper):
    """Return every combination of lower/upper X, Y and Z bounds."""
    lower, upper = (np.asarray(lower, dtype=float), np.asarray(upper, dtype=float))
    return box_corners((lower + upper) / 2, upper - lower)


def _roots(world_path):
    """Find model directories beside the world and in GAZEBO_MODEL_PATH."""
    siblings = os.path.join(os.path.dirname(os.path.dirname(world_path)), "models")
    # Search local model folders first, then configured Gazebo model directories.
    roots = [siblings] + os.environ.get("GAZEBO_MODEL_PATH", "").split(":")
    results = []
    for root in roots:
        if root and os.path.isdir(root):
            results.append(root)
    return results


def _resolve(uri, roots):
    """model://name/rest as a real path, or None if no root holds it."""
    # Try the model URI under each search directory until a file exists.
    for root in roots:
        path = os.path.join(root, uri.replace("model://", "", 1))
        if os.path.exists(path):
            return path
    return None


def _model_sdf(uri, roots):
    """Read model.config to find the model SDF filename."""
    directory = _resolve(uri, roots)
    if directory is None:
        return None
    # Read the model manifest to locate its SDF definition.
    return os.path.join(
        directory,
        ET.parse(os.path.join(directory, "model.config")).getroot().findtext("sdf"),
    )


def _collada_unit(path):
    """Metres per unit, as COLLADA declares in its own header."""
    if not path.endswith(".dae"):
        return 1.0
    # Read the COLLADA unit scale so mesh coordinates become metres.
    unit = ET.parse(path).getroot().find("{*}asset/{*}unit")
    if unit is not None:
        return float(unit.get("meter", 1.0))
    else:
        return 1.0


def _geometry_corners(geometry, roots):
    """Approximate supported collision geometry with local box corners."""
    # Approximate each supported collision shape with the corners of a local box.
    box = geometry.find("box")
    if box is not None:
        extents = np.fromstring(box.findtext("size"), sep=" ")
        return _corners(-extents / 2, extents / 2)
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius, length = (
            float(cylinder.findtext("radius")),
            float(cylinder.findtext("length")),
        )
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
            raise FileNotFoundError(f"Cannot resolve mesh {mesh.findtext('uri')}")
        # Apply both the mesh scale and the file's declared units.
        scale = np.fromstring(mesh.findtext("scale", "1 1 1"), sep=" ") * _collada_unit(
            path
        )
        bounds = trimesh.load(path, force="mesh").bounds * scale
        return _corners(bounds[0], bounds[1])
    return None


def _collision_points(model, pose, roots):
    """Every collision corner of a model, in the world frame."""
    points = []
    if model.find("model") is not None or model.find("include") is not None:
        raise ValueError("Nested SDF models need frame resolution before loading")
    for link in model.findall("link"):
        # Combine model, link, and collision poses to place corners in the world.
        link_pose = pose @ _pose(link)
        for collision in link.iter("collision"):
            corners = _geometry_corners(collision.find("geometry"), roots)
            if corners is not None:
                points.append(transform_points(link_pose @ _pose(collision), corners))
    if points:
        return np.vstack(points)
    else:
        return np.empty((0, 3))


def _models(world, roots):
    """(pose source, model, label, name) for every model in the world."""
    # Resolve included model files before processing models embedded in the world.
    for element in world.findall("include"):
        uri = element.findtext("uri")
        sdf = _model_sdf(uri, roots)
        if sdf is None:
            raise FileNotFoundError(f"Cannot resolve {uri} under {roots}")
        label = uri.rsplit("/", 1)[-1]
        yield (
            element,
            ET.parse(sdf).getroot().find("model"),
            label,
            element.findtext("name") or label,
        )
    for model in world.findall("model"):
        yield (model, model, model.get("name"), model.get("name"))


def _scannet_label(model):
    """A model's ScanNet200 class, so sim and scan graphs share one vocabulary."""
    # Require a shared class label so simulation and scan objects use the same names.
    label = scannet_class(model)
    if label is None:
        raise ValueError(
            f"Gazebo model {model!r} has no ScanNet200 class; add it under gazebo: in {LABELS}"
        )
    return label


def load_world(path):
    """Read initial collision geometry in Gazebo world coordinates, in metres."""
    roots = _roots(path)
    # This is the initial world-file snapshot, not live simulator state.
    world = ET.parse(path).getroot().find("world")
    if world is None:
        raise ValueError("SDF file has no world")
    instances = []
    for element, model, label, name in _models(world, roots):
        pose = _pose(element)
        if element is not model:
            pose = pose @ _pose(model)
        points = _collision_points(model, pose, roots)
        # Keep models with collision geometry and map their labels to shared classes.
        if len(points):
            instances.append(Instance(_scannet_label(label), points, name=name))
    return instances
