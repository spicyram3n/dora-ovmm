"""Checks the Gazebo source against ground truth apartment.world states, not
against anything this file made up.

From core/, either of:
    python3 -m scene_graph.test_gazebo          # also prints the graph
    python3 -m pytest scene_graph/test_gazebo.py -p no:anyio

The -p no:anyio is this image's pytest 6.2.5 being older than the installed
anyio plugin expects; it has nothing to do with these tests.
"""

from pathlib import Path

from scene_graph import gazebo, graph as sg
from scene_graph.instance import is_structure

WORLD = (Path(__file__).resolve().parents[2]
         / "ros2_ws/src/tmc_gazebo/tmc_gazebo_worlds/worlds/apartment.world")

# Where every object in the apartment sits, by the pose the world file gives
# it. The graph is told none of this: it has only points and labels.
PLACEMENTS = {
    "nasal_cupsul": ("on", "wagon"),
    "wallet": ("in", "living_sideboard"),
    "apple_01": ("in", "high_shelf01"),
    "pear_01": ("in", "high_shelf03"),
    "banana_01": ("in", "high_shelf03"),
    "hsr_pringles_01": ("in", "high_shelf02"),
    "hsr_pringles_02": ("on", "high_table01"),
    "hsr_orange_02": ("on", "high_table01"),
}


def apartment():
    return sg.build(gazebo.load_world(WORLD))


def named(scene, name):
    return next(node for node, data in scene.nodes(data=True) if data["name"] == name)


def test_every_object_lands_on_the_right_furniture():
    scene = apartment()
    for name, expected in PLACEMENTS.items():
        target, relation = sg.location_of(scene, named(scene, name))
        assert (relation, scene.nodes[target]["name"]) == expected, name


def test_structure_is_not_a_destination():
    """Walls, doors and door stoppers must not survive as furniture: the LLM
    picks from whatever is left, and it can only pick a real place to stand."""
    scene = apartment()
    assert not any(is_structure(data["label"]) for data in scene.nodes.values())
    assert len(sg.furniture(scene)) == 16


def test_meshes_are_scaled_by_their_own_collada_unit():
    """sofa_seat.dae is authored in decimetres. Taking its vertices at face
    value gives a 12 metre sofa, which then swallows half the flat."""
    sofa = next(i for i in gazebo.load_world(WORLD) if i.label == "sofa-fix")
    assert all(0.3 < side < 2.0 for side in sofa.dimensions), sofa.dimensions


def test_coordinates_are_in_the_robots_odom_frame():
    """The robot spawns at world (5.0, 6.6), and odom starts where the robot
    did, so a goal built from world coordinates would be ~8 m out."""
    scene = apartment()
    centroid = scene.nodes[named(scene, "living_sideboard")]["centroid"]
    assert abs(centroid[0] + 3.77) < 0.05 and abs(centroid[1] + 6.19) < 0.05


def test_duplicate_labels_resolve_to_the_nearest_instance():
    """Two pringles cans: one on a shelf at odom y ~6.4, one on the high table
    at y ~-2.7. Which one the robot means depends on where the robot is."""
    scene = apartment()
    by_shelf = sg.find_object(scene, "hsr_pringles", near=(-0.7, 6.4))
    by_table = sg.find_object(scene, "hsr_pringles", near=(-0.4, -2.7))
    assert scene.nodes[by_shelf]["name"] == "hsr_pringles_01"
    assert scene.nodes[by_table]["name"] == "hsr_pringles_02"


if __name__ == "__main__":
    scene = apartment()
    print(f"{scene.number_of_nodes()} nodes: {len(sg.furniture(scene))} furniture, "
          f"{len(sg.objects(scene))} objects\n")
    for node, data in sorted(sg.objects(scene).items(), key=lambda kv: kv[1]["name"]):
        target, relation = sg.location_of(scene, node)
        where = f"{relation} {scene.nodes[target]['name']}" if target is not None else "unattached"
        print(f"  {data['name']:20} {where}")

    print()
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"  {name} ok")
