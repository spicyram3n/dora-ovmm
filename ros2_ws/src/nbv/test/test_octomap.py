"""What build_octomap.py relies on from octomap-python. No ROS needed:

    python3 -m pytest -p no:anyio ros2_ws/src/nbv/test
"""

import numpy as np
import pytest

octomap = pytest.importorskip("octomap")
FREE, OCCUPIED, UNKNOWN = 0, 1, -1


def test_a_hit_beyond_the_sphere_frees_the_space_it_looked_through():
    tree = octomap.OcTree(0.05)
    # Camera at x = -1 sees a wall at x = 1, straight through a sphere at the origin.
    tree.insertPointCloud(np.array([[1.0, 0.0, 0.0]]), np.array([-1.0, 0.0, 0.0]), maxrange=2.49)
    labels = tree.getLabels(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    assert labels.tolist() == [FREE, OCCUPIED]


def test_a_ray_with_no_return_is_free_but_not_occupied():
    tree = octomap.OcTree(0.05)
    # build_octomap.py puts these at MAX_RANGE and inserts with FREE_RANGE just under it.
    tree.insertPointCloud(np.array([[2.5, 0.0, 0.0]]), np.array([0.0, 0.0, 0.0]), maxrange=2.49)
    labels = tree.getLabels(np.array([[1.0, 0.0, 0.0], [2.5, 0.0, 0.0], [0.0, 2.0, 0.0]]))
    assert labels.tolist() == [FREE, UNKNOWN, UNKNOWN]


def test_occupied_cells_come_back_at_their_centre_with_their_probability():
    tree = octomap.OcTree(0.05)
    hit = np.array([0.33, -0.21, 0.47])
    tree.insertPointCloud(np.array([hit]), np.array([0.0, 0.0, 0.0]), maxrange=2.49)
    occupied, _ = tree.extractPointCloud()
    # pyoctomap 1.2.2 returns these half a cell off, which is why this is not pyoctomap.
    assert np.allclose(occupied, [tree.keyToCoord(tree.coordToKey(hit))])
    assert tree.search(occupied[0]).getOccupancy() > 0.5
