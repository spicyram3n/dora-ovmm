import unittest
from types import SimpleNamespace
from copy import deepcopy
from moveit_msgs.msg import PlanningScene
from core.grasping.scene_payload import compact


class ScenePayload(unittest.TestCase):
    def test_full_snapshot_kept_and_only_identical_diff_omitted(self):
        initial=PlanningScene();initial.world.octomap.octomap.data=[1,2,3]
        same=deepcopy(initial);same.is_diff=True
        changed=deepcopy(same);changed.world.octomap.octomap.data=[4,5,6]
        full=deepcopy(changed);full.is_diff=False
        solution=SimpleNamespace(start_scene=initial,sub_trajectory=[
            SimpleNamespace(scene_diff=s) for s in [same,changed,full]])
        compact(solution)
        self.assertEqual(list(initial.world.octomap.octomap.data),[1,2,3])
        self.assertFalse(same.world.octomap.octomap.data)
        self.assertEqual(list(changed.world.octomap.octomap.data),[4,5,6])
        self.assertEqual(list(full.world.octomap.octomap.data),[4,5,6])
