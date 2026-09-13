import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from moveit_msgs.msg import PlanningScene
import core.grasping.pick as p

class MapReadiness(unittest.TestCase):
    def test_waits_for_real_map_before_freezing(self):
        empty=PlanningScene();ready=PlanningScene();ready.world.octomap.octomap.data=[1]
        responses=iter([NS(success=True),NS(),NS(scene=empty),NS(scene=ready),NS(valid=True)])
        events=[]
        def call(*args):
            events.append(args[2]);return next(responses)
        with patch.object(p,'call',side_effect=call),patch.object(p,'depth_relay',side_effect=lambda n,v:events.append(v)),patch.object(p.time,'sleep'):
            p.model_target(None,np.array([[0,0,0],[.05,.05,.1]]))
        self.assertEqual(events.count('/get_planning_scene'),2)
        self.assertEqual(events[-2:],['/check_state_validity',False])

    def test_empty_map_timeout_does_not_freeze_as_ready(self):
        responses=iter([NS(success=True),NS(),NS(scene=PlanningScene())])
        with patch.object(p,'call',side_effect=lambda *a:next(responses)),patch.object(p,'depth_relay') as relay,patch.object(p.time,'monotonic',side_effect=[0,31]):
            with self.assertRaisesRegex(RuntimeError,'No fresh depth map'):
                p.model_target(None,np.array([[0,0,0],[.05,.05,.1]]))
            relay.assert_called_once_with(None,True)
