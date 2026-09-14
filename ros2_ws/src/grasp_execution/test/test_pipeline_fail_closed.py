"""The public pipeline delegates motion once and preserves grasp outcomes."""
import unittest
from unittest.mock import patch, Mock

from core import run_pipeline as pipeline


class PipelineEntryTests(unittest.TestCase):
    def test_execution_uses_pick_for_every_mode_without_preview_or_retry(self):
        for mode in ('auto', 'grasp', 'pickup'):
            for code, expected in ((0, 0), (3, 3), (1, 1), (-15, 1), (7, 1)):
                with self.subTest(mode=mode, code=code), \
                        patch.object(pipeline.subprocess, 'run', return_value=Mock(returncode=code)) as run, \
                        patch.object(pipeline, 'grab_rgbd') as camera, \
                        patch.object(pipeline, 'process') as preview:
                    self.assertEqual(pipeline.main(['spray bottle'], execute=True, mode=mode), expected)
                    run.assert_called_once()
                    self.assertEqual(run.call_args.args[0], [pipeline.sys.executable, '-m',
                                     'core.grasping.pick', 'spray bottle', '--mode', mode])
                    self.assertEqual(run.call_args.kwargs['cwd'], pipeline.ROOT)
                    self.assertEqual(run.call_args.kwargs['env']['HSR_GRASP_DIAGNOSTICS'],
                                     str(pipeline.TARGETS_DIR.resolve() / 'spray_bottle'))
                    camera.assert_not_called()
                    preview.assert_not_called()

    def test_invalid_execution_inputs_never_launch_motion(self):
        with patch.object(pipeline.subprocess, 'run') as run:
            for prompts, gripper, mode in ((['a', 'b'], 'hsrc_hand', 'auto'),
                                           ([], 'hsrc_hand', 'auto'),
                                           (['a'], 'other', 'auto'),
                                           (['a'], 'hsrc_hand', 'invalid')):
                with self.assertRaises(ValueError):
                    pipeline.main(prompts, gripper, execute=True, mode=mode)
            run.assert_not_called()

    def test_preview_shares_camera_frame_and_never_executes(self):
        with patch.object(pipeline, 'grab_rgbd', return_value=(1, 2, 3, 4)) as camera, \
                patch.object(pipeline, 'process') as process, \
                patch.object(pipeline.subprocess, 'run') as run:
            self.assertEqual(pipeline.main(['a', 'b']), 0)
            camera.assert_called_once()
            self.assertEqual(process.call_count, 2)
            run.assert_not_called()

    def test_preview_failure_is_reported_and_other_targets_continue(self):
        with patch.object(pipeline, 'grab_rgbd', return_value=(1, 2, 3, 4)), \
                patch.object(pipeline, 'process', side_effect=[RuntimeError('missing'), None]) as process:
            self.assertEqual(pipeline.main(['a', 'b']), 1)
            self.assertEqual(process.call_count, 2)

    def test_obsolete_motion_flags_are_rejected(self):
        for flag in ('--lift', '--retreat', '--attempts'):
            with patch('sys.argv', ['run_pipeline.py', 'can', '--execute', flag, '1']), \
                    patch.object(pipeline.subprocess, 'run') as run:
                with self.assertRaises(SystemExit) as error:
                    pipeline.cli()
                self.assertEqual(error.exception.code, 2)
                run.assert_not_called()
