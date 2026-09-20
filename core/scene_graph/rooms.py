"""Partition furniture into rooms using the map, not the labels.

The reasoner only ever sees labels and centroids, so asking it to group furniture
into rooms asks it to guess where the walls are. On the lab graph it guessed
wrong twice over: it split one desk row into `office` and `dining`, and it merged
a room that is only reachable along a corridor into `office`.

So the partition is geometric and the naming is semantic, and they are separate
steps. Here we cluster furniture by how far apart they are *along the floor the
robot actually drove* -- a wall between two desks makes that distance long even
when they are a metre apart in a straight line. `core.reasoner.deepseek.name`
then puts a name on each cluster, which is the part that needs to know what an
oven means.

Nothing here is specific to one building: it reads any Nav2 map, needs no
enclosed rooms, and tolerates a map that covers only the route the robot took.
The one parameter is `spacing`, in metres of floor; `sweep` prints the cluster
count against it so a new building can be set up in one command.
"""

import numpy as np
from scipy.cluster.hierarchy import fcluster, linkage
from scipy.ndimage import binary_closing, distance_transform_edt, label
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial.distance import squareform

from core.scene_graph import graph as sg
from core.utils.occupancy import to_cells

# Trinary PGM values: Nav2 writes free at 254, unknown at 205 and occupied at 0.
FREE, OCCUPIED = 250, 100
# An obstacle blob longer than this is structure the robot must walk around; a
# shorter one is a table leg or a bin, which sits *inside* a room and must not
# cut it in two. Measured on the blob's longest side, so a thin wall still counts.
WALL_SPAN = 1.0
# Floor distance at which two pieces of furniture stop counting as one room.
# Not a property of this lab: it is how far apart things in one room can be.
# Run `sweep` on a new map and read it off the plateau.
SPACING = 6.0
# Eight-connected: a wall one cell thick must not leak through its diagonals.
NEIGHBOURS = ((0, 1, 1.0), (1, 0, 1.0), (1, 1, 2 ** 0.5), (1, -1, 2 ** 0.5))


def walls(image, resolution):
    """The occupied cells that are structure rather than clutter."""
    blobs, count = label(image < OCCUPIED, structure=np.ones((3, 3)))
    spans = []
    for index in range(1, count + 1):
        rows, columns = np.nonzero(blobs == index)
        # A blob's longest side says whether it is a wall or a chair leg.
        spans.append(max(np.ptp(rows), np.ptp(columns)) * resolution)
    structural = np.nonzero(np.asarray(spans) > WALL_SPAN)[0] + 1
    return np.isin(blobs, structural)


def floor(image, resolution):
    """The largest connected piece of mapped floor, with the walls taken out.

    Unknown cells are not floor: on a map built from one drive most of the
    image is unknown, and treating it as walkable would let the clustering
    step straight through a wall into the next room."""
    structure = walls(image, resolution)
    # Close pinholes the laser left in open floor, then add back the clutter
    # cells: a bin standing in a room does not divide it.
    space = binary_closing(image > FREE, np.ones((3, 3))) & ~structure
    space |= (image < OCCUPIED) & ~structure
    pieces, count = label(space, structure=np.ones((3, 3)))
    if count == 0:
        raise ValueError("the map has no free space")
    # Furniture the drive never reached is attached to the nearest floor below.
    largest = np.bincount(pieces.ravel())[1:].argmax() + 1
    return pieces == largest


def _grid_graph(space, resolution):
    """(sparse adjacency, cell -> node index) over the walkable cells."""
    index = np.full(space.shape, -1, dtype=int)
    index[space] = np.arange(int(space.sum()))
    height, width = space.shape
    rows, columns, weights = [], [], []
    for down, right, cost in NEIGHBOURS:
        # Line every cell up against its neighbour in this direction at once.
        here = index[max(0, -down):height - max(0, down),
                     max(0, -right):width - max(0, right)]
        there = index[max(0, down):height - max(0, -down),
                      max(0, right):width - max(0, -right)]
        both = (here >= 0) & (there >= 0)
        rows.extend(here[both])
        columns.extend(there[both])
        weights.extend([cost * resolution] * int(both.sum()))
    size = int(space.sum())
    return coo_matrix((weights, (rows, columns)), shape=(size, size)).tocsr(), index


def _cell(space, index, nearest, resolution, origin, point):
    """The walkable graph node for a map point, snapped to the floor beside it."""
    column, row = np.floor(to_cells(point[:2], resolution, origin)).astype(int)
    row = int(np.clip(row, 0, space.shape[0] - 1))
    column = int(np.clip(column, 0, space.shape[1] - 1))
    if not space[row, column]:
        row, column = int(nearest[0][row, column]), int(nearest[1][row, column])
    return index[row, column]


def _seeds(graph, space, index, resolution, origin):
    """(node ids, their cell in the walkable graph) for every piece of furniture."""
    # Furniture stands against walls, so its centre often is not walkable; the
    # nearest walkable cell is the floor in front of it.
    nearest = distance_transform_edt(~space, return_distances=False, return_indices=True)
    ids, cells = [], []
    for node_id, data in sg.furniture(graph).items():
        ids.append(node_id)
        cells.append(_cell(space, index, nearest, resolution, origin, sg.footprint(data)[0][:2]))
    return ids, cells


def room_at(graph, image, resolution, origin, point, limit=None):
    """The room of the furniture nearest `point` along the floor, or None.

    `partition` divides furniture by floor distance because a straight line runs
    through walls. Which room the robot is standing in has to be asked the same
    way, or the division and the question disagree in every doorway.

    `limit`, in metres of floor, answers None rather than naming a room the robot
    is nowhere near. Leave it off to always answer. It is not `SPACING`, which
    measures furniture against furniture: on lab_20260919_wheel the floor sits a
    median 4.4 m and at most 21.4 m from the nearest furniture, so a 6 m limit
    silences 39% of it and a 12 m one 12%.
    """
    space = floor(image, resolution)
    adjacency, index = _grid_graph(space, resolution)
    ids, cells = _seeds(graph, space, index, resolution, origin)
    if not ids:
        return None
    nearest = distance_transform_edt(~space, return_distances=False, return_indices=True)
    start = _cell(space, index, nearest, resolution, origin, point)
    walk = dijkstra(adjacency, indices=[start], directed=False)[0][cells]
    closest = int(np.argmin(walk))
    # Further than a room is wide: the robot is between rooms, not inside one.
    if not np.isfinite(walk[closest]) or (limit is not None and walk[closest] > limit):
        return None
    return graph.nodes[ids[closest]]["room"]


def distances(graph, image, resolution, origin):
    """(node ids, square matrix of floor distance in metres between them)."""
    space = floor(image, resolution)
    adjacency, index = _grid_graph(space, resolution)
    ids, cells = _seeds(graph, space, index, resolution, origin)
    if not ids:
        return ([], np.zeros((0, 0)))
    matrix = dijkstra(adjacency, indices=cells, directed=False)[:, cells]
    # Dijkstra over an undirected graph still returns the two directions
    # separately; average them so the result is a metric scipy will accept.
    matrix = (matrix + matrix.T) / 2
    np.fill_diagonal(matrix, 0.0)
    if not np.isfinite(matrix).all():
        raise ValueError("some furniture is cut off from the rest of the floor")
    return (ids, matrix)


def _tree(matrix):
    # Average linkage: one outlying chair cannot pull two rooms together the way
    # single linkage would, nor split a long room the way complete linkage does.
    return linkage(squareform(matrix, checks=False), method="average")


def partition(graph, image, resolution, origin, spacing=SPACING):
    """{furniture node id: cluster index}, clusters numbered from 0."""
    ids, matrix = distances(graph, image, resolution, origin)
    if not ids:
        return {}
    clusters = fcluster(_tree(matrix), spacing, criterion="distance")
    return {node_id: int(cluster) - 1 for node_id, cluster in zip(ids, clusters)}


def sweep(graph, image, resolution, origin, limits=(1.0, 15.0), step=0.1):
    """[(clusters, from, to)] over `spacing`, widest run of one count first.

    How to choose `spacing` for a new building: a range of spacings that all
    give the same number of rooms is a real division of the floor, not an
    artefact of the number. Pick from the middle of a wide run. Ignore the run
    at the top, which is the whole building falling into two halves."""
    _, matrix = distances(graph, image, resolution, origin)
    tree = _tree(matrix)
    values = np.arange(limits[0], limits[1], step)
    counts = [int(fcluster(tree, value, criterion="distance").max()) for value in values]
    runs, start = [], 0
    for position in range(1, len(counts) + 1):
        if position == len(counts) or counts[position] != counts[start]:
            runs.append((counts[start], float(values[start]), float(values[position - 1])))
            start = position
    return sorted(runs, key=lambda run: run[2] - run[1], reverse=True)
