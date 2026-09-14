import importlib.util
import unittest
from unittest.mock import Mock, patch

import py_trees
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.utilities import perform_substitutions
from core.pipeline import actions, mission_tree


class MissionTests(unittest.TestCase):
    def test_tree_stops_after_one_pick_without_homing_again(self):
        for success in (True, False):
            calls = []
            steps = {name: (lambda n=name: calls.append(n) or (success if n == 'pick' else True))
                     for name in ('ready', 'home', 'find', 'park', 'pause', 'pick')}
            tree = mission_tree.build(steps)
            for _ in range(20):
                tree.tick_once()
                if tree.status != py_trees.common.Status.RUNNING:
                    break
            self.assertEqual(calls, ['ready', 'home', 'find', 'park', 'pause', 'pick'])
            self.assertEqual(mission_tree.exit_code(tree), 0 if success else 4)

    def test_navigation_and_search_only_never_pick(self):
        for navigate_only in (True, False):
            calls = []
            steps = {name: (lambda n=name: calls.append(n) or True)
                     for name in ('ready', 'home', 'choose', 'go', 'find', 'park', 'pause', 'pick')}
            tree = mission_tree.build(steps, grasp=False, navigate_only=navigate_only)
            for _ in range(20):
                tree.tick_once()
                if tree.status != py_trees.common.Status.RUNNING:
                    break
            self.assertNotIn('pick', calls)
            self.assertNotIn('pause', calls)
            self.assertEqual(mission_tree.exit_code(tree), 0)

    def test_pick_dispatch_preserves_mode_and_outcome(self):
        for mode in ('grasp', 'auto', 'pickup'):
            for code, status in ((0, actions.PICKED), (3, actions.GRASPED), (1, actions.NOT_PICKED)):
                with patch.object(actions.subprocess, 'run', return_value=Mock(returncode=code)) as run:
                    self.assertEqual(actions.pick_up('spray bottle', mode), status)
                    self.assertEqual(run.call_args.args[0][-4:], ['core.grasping.pick', 'spray bottle', '--mode', mode])
                    run.assert_called_once()

    def test_contact_hold_is_successful_in_tree(self):
        steps = mission_tree.mission_steps(Mock(), {}, 'spray bottle', 'unused', 3, 6, 180, True, mode='grasp')
        with patch.object(actions, 'pick_up', return_value=actions.GRASPED) as pick:
            self.assertTrue(steps['pick']())
            pick.assert_called_once_with('spray bottle', mode='grasp')

    def test_search_launch_routes_to_tree_with_grasp_mode(self):
        spec = importlib.util.spec_from_file_location('search_launch', actions.ROOT / 'launch/search.launch.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        description = module.generate_launch_description()
        context = LaunchContext()
        context.launch_configurations.update(target='spray bottle', mode='grasp')
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        processes = [e for e in description.entities if isinstance(e, ExecuteProcess)]
        commands = [[perform_substitutions(context, part) for part in p.cmd] for p in processes]
        missions = [cmd for cmd in commands if 'core.pipeline.mission_tree' in cmd]
        self.assertEqual(len(missions), 1)
        self.assertEqual(missions[0][-2:], ['--mode', 'grasp'])
        self.assertFalse(any('run_pipeline.py' in str(cmd) for cmd in commands))
