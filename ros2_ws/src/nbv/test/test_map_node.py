"""The map node's region query and OctoMap message, without starting the node:

    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    python3 -m pytest -p no:anyio ros2_ws/src/nbv/test
"""

import importlib.util
import os
import tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nbv.srv")
octomap = pytest.importorskip("octomap")
SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "build_octomap.py"
spec = importlib.util.spec_from_file_location("build_octomap", SCRIPT)
node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(node)


def wall(x, sensor, tree):
    """A 0.4 m square wall at `x`, seen from `sensor`."""
    points = []
    for y in np.arange(-0.2, 0.21, 0.02):
        for z in np.arange(-0.2, 0.21, 0.02):
            points.append([x, y, z])
    tree.insertPointCloud(np.array(points), np.array(sensor), maxrange=2.49)


def test_an_unseen_sphere_is_all_unknown():
    tree = octomap.OcTree(0.05)
    unknown, total = node.unknown_in_sphere(tree, [0.0, 0.0, 0.0], 0.3)
    assert total > 0
    assert unknown == total


def test_looking_through_a_sphere_makes_part_of_it_known():
    tree = octomap.OcTree(0.05)
    wall(1.0, [-1.0, 0.0, 0.0], tree)
    unknown, total = node.unknown_in_sphere(tree, [0.0, 0.0, 0.0], 0.3)
    assert 0 < unknown < total


def test_a_sphere_with_no_cells_reports_none():
    tree = octomap.OcTree(0.05)
    assert node.unknown_in_sphere(tree, [0.0, 0.0, 0.0], -1.0) == (0, 0)


def test_the_octomap_message_holds_exactly_the_map():
    tree = octomap.OcTree(0.05)
    wall(1.0, [-1.0, 0.0, 0.0], tree)
    message = node.OctomapBuilder.octomap(SimpleNamespace(tree=tree), node.Header(frame_id="map"))
    assert message.binary and message.id == "OcTree" and message.resolution == 0.05
    # RViz's reader takes the data alone; rebuild a file around it to read it back here.
    header = f"# Octomap OcTree binary file\nid OcTree\nsize {tree.size()}\nres 0.05\ndata\n"
    path = tempfile.mktemp(suffix=".bt")
    with open(path, "wb") as out:
        out.write(header.encode() + bytes(message.data))
    copy = octomap.OcTree(0.05)
    try:
        assert copy.readBinary(path.encode())
    finally:
        os.remove(path)
    probe = np.random.default_rng(0).uniform(-1.5, 1.5, (20000, 3))
    assert np.array_equal(tree.getLabels(probe), copy.getLabels(probe))
