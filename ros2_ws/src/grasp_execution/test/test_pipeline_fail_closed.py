import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'core'))
import run_pipeline


class PipelineFailureTest(unittest.TestCase):
    def test_enabled_base_recovery_is_blocked_after_target_separation(self):
        with tempfile.TemporaryDirectory() as directory:
            grasp_file = Path(directory) / 'grasps.yaml'
            grasp_file.write_text('target_geometry: {radius: 0.033}\n')
            process = MagicMock()
            process.__enter__.return_value = process
            process.stdout = ['separating only the target\n',
                              'none of the 50 candidate grasps were reachable\n']
            process.wait.return_value = 1
            with patch.object(run_pipeline, 'grab_rgbd', return_value=(None,)*4), \
                    patch.object(run_pipeline, 'process', return_value=grasp_file), \
                    patch.object(run_pipeline.subprocess, 'Popen', return_value=process), \
                    patch.object(run_pipeline.subprocess, 'run') as recovery:
                self.assertFalse(run_pipeline.main(['can'], 'hsrc_hand', execute=True, recover_base=True))
                recovery.assert_not_called()

    def test_no_reachable_candidate_invokes_recovery_then_requests_fresh_batch(self):
        with tempfile.TemporaryDirectory() as directory:
            grasp_file = Path(directory) / 'grasps.yaml'
            grasp_file.write_text('target_geometry: {radius: 0.033}\n')
            process = MagicMock()
            process.__enter__.return_value = process
            process.stdout = ['none of the 50 candidate grasps were reachable\n']
            process.wait.return_value = 1
            with patch.object(run_pipeline, 'grab_rgbd', return_value=(None,)*4), \
                    patch.object(run_pipeline, 'process', return_value=grasp_file), \
                    patch.object(run_pipeline.subprocess, 'Popen', return_value=process), \
                    patch.object(run_pipeline.subprocess, 'run') as recovery:
                self.assertIsNone(run_pipeline.main(['can'], 'hsrc_hand', execute=True, recover_base=True))
                self.assertEqual(recovery.call_count, 1)
                self.assertTrue(recovery.call_args.args[0][1].endswith('grasp_recovery.py'))

    def test_fresh_batch_retry_is_blocked_after_target_separation(self):
        with tempfile.TemporaryDirectory() as directory:
            grasp_file = Path(directory) / 'grasps.yaml'
            grasp_file.write_text('target_geometry: {radius: 0.033}\n')
            for separated in (False, True):
                with self.subTest(separated=separated):
                    process = MagicMock()
                    process.__enter__.return_value = process
                    process.stdout = (['separating only the target\n'] if separated else []) + [
                        'none of the 50 candidate grasps were reachable\n']
                    process.wait.return_value = 0
                    with patch.object(run_pipeline, 'grab_rgbd', return_value=(None,) * 4), \
                            patch.object(run_pipeline, 'process', return_value=grasp_file), \
                            patch.object(run_pipeline.subprocess, 'Popen', return_value=process), \
                            patch.object(run_pipeline.subprocess, 'run') as finish:
                        result = run_pipeline.main(['can'], 'hsrc_hand', execute=True)
                        self.assertIs(result, False if separated else None)
                        finish.assert_not_called()

    def test_detection_failure_cannot_execute_saved_grasps(self):
        with patch.object(run_pipeline, 'grab_rgbd', return_value=(None,) * 4), \
                patch.object(run_pipeline, 'process', side_effect=RuntimeError('No object')), \
                patch.object(run_pipeline.subprocess, 'Popen') as approach, \
                patch.object(run_pipeline.subprocess, 'run') as finish:
            self.assertFalse(run_pipeline.main(['can'], 'hsrc_hand', execute=True))
            approach.assert_not_called()
            finish.assert_not_called()

    def test_clean_launch_exit_without_reached_contact_cannot_close(self):
        with tempfile.TemporaryDirectory() as directory:
            grasp_file = Path(directory) / 'grasps.yaml'
            grasp_file.write_text('target_geometry: {radius: 0.033}\n')
            process = MagicMock()
            process.__enter__.return_value = process
            process.stdout = ['No reachable candidate\n']
            process.wait.return_value = 0
            with patch.object(run_pipeline, 'grab_rgbd', return_value=(None,) * 4), \
                    patch.object(run_pipeline, 'process', return_value=grasp_file), \
                    patch.object(run_pipeline.subprocess, 'Popen', return_value=process), \
                    patch.object(run_pipeline.subprocess, 'run') as finish:
                self.assertFalse(run_pipeline.main(['can'], 'hsrc_hand', execute=True))
                finish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
