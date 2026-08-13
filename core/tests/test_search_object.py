"""Guards the seam between the scene graph and the LLM.

The bug this exists for: search_object collected the search order into a list,
which pulled the LLM entry even when the graph had already answered. A
scenario B lookup that should be instant instead waited a minute on
deepseek-reasoner for a guess nobody used.

From core/:
    python3 -m pytest test_search_object.py -p no:anyio
    python3 test_search_object.py
"""

from pathlib import Path

import search_object
from navigation import standoff
from scene_graph import graph as sg
from spatial_reasoning import query_sg

GRAPH = Path(__file__).resolve().parents[2] / "outputs/scene_graph/apartment/graph.json"


def _no_llm_allowed():
    raise AssertionError("the LLM was contacted during a scenario B search")


def test_known_object_never_reaches_the_llm():
    """hsr_pringles is in the graph, so its furniture is already known and
    nothing should ask a model about it."""
    original = query_sg.get_client
    query_sg.get_client = _no_llm_allowed
    try:
        search_object.main("hsr_pringles", None, "apartment", dry_run=True, top_k=3)
    finally:
        query_sg.get_client = original


def test_named_furniture_never_reaches_the_llm():
    original = query_sg.get_client
    query_sg.get_client = _no_llm_allowed
    try:
        search_object.main("", "high_table01", "apartment", dry_run=True, top_k=3)
    finally:
        query_sg.get_client = original


def test_the_search_order_stays_lazy():
    """targets() must be a generator. If it is ever turned back into a list,
    building it is what calls the model."""
    scene = sg.load(GRAPH)
    produced = search_object.targets(scene, "hsr_pringles", None, None, (0.0, 0.0), 3)
    assert hasattr(produced, "__next__"), "targets() must not be materialised"
    assert next(produced).source == "scene_graph"


def test_a_coffee_table_is_approached_from_its_ends_not_through_its_sofas():
    """kitchen_lowtable sits 0.9 m from sofa01 and 1.0 m from sofa02, and is
    wider than it is deep -- so the smallest-radius bearing, the one
    candidates() prefers, points straight into a sofa. Nav2 does not save us:
    it finds a path to such a pose, then stops against the sofa inside its
    0.25 m goal tolerance and reports success."""
    scene = sg.load(GRAPH)
    table = scene.nodes[4]
    blockers = [(f["centroid"], f["dimensions"])
                for node, f in sg.furniture(scene).items() if node != 4]

    assert any(standoff.blocks(pose, blockers)
               for pose in standoff.candidates(table["centroid"], table["dimensions"]))

    poses = standoff.candidates(table["centroid"], table["dimensions"], blockers=blockers)
    assert poses, "the table must still be approachable from its ends"
    assert not any(standoff.blocks(pose, blockers) for pose in poses)
    # the two best survivors are on the table's x ends, level with its centre
    assert all(abs(y - table["centroid"][1]) < 0.3 for _, y, _ in poses[:2])


def test_every_piece_keeps_somewhere_to_stand():
    """A filter that empties the list for some piece would strand the robot."""
    scene = sg.load(GRAPH)
    pieces = sg.furniture(scene)
    for node, data in pieces.items():
        blockers = [(f["centroid"], f["dimensions"]) for other, f in pieces.items()
                    if other != node]
        assert standoff.candidates(data["centroid"], data["dimensions"],
                                   blockers=blockers), data["name"]


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"{name} ok\n")
