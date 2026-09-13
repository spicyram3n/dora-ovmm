import unittest
from unittest.mock import Mock, patch
from core.grasping.contact_closure import contact_state, wait_for_hold


class ContactFeedback(unittest.TestCase):
    def test_higher_preload_retains_hard_stop(self):
        self.assertTrue(contact_state(.18,.18,.17))
        with self.assertRaises(RuntimeError):
            contact_state(.18,.21,.17)
        with self.assertRaises(ValueError):
            contact_state(.19,.19,.19)

    @patch('core.grasping.contact_closure.time.sleep')
    def test_lift_then_slip_is_failure(self, sleep):
        with self.assertRaisesRegex(RuntimeError, 'slipped'):
            wait_for_hold(Mock(side_effect=[(.12, .12), (.0, .0)]),
                          Mock(side_effect=[0., 1.]))

    @patch('core.grasping.contact_closure.time.sleep')
    def test_sustained_contact_uses_simulation_time(self, sleep):
        read = Mock(return_value=(.12, .12))
        wait_for_hold(read, Mock(side_effect=[0., 1., 3., 5.]))
        self.assertEqual(read.call_count, 4)

    def test_paused_simulation_times_out(self):
        with self.assertRaisesRegex(RuntimeError, 'Timed out'):
            wait_for_hold(lambda: (.12, .12), lambda: 0., timeout=0.)

    def test_one_finger_is_not_a_grasp(self):
        self.assertFalse(contact_state(.12, .01, .06))
        self.assertFalse(contact_state(.01, .12, .06))

    def test_bilateral_contact(self):
        self.assertTrue(contact_state(.08, .13, .06))

    def test_excessive_contact_cannot_report_success(self):
        with self.assertRaises(RuntimeError):
            contact_state(.10, .25, .06)

    def test_invalid_feedback(self):
        with self.assertRaises(ValueError):
            contact_state(float('nan'), .12, .06)

    def test_grasp_only_threshold(self):
        self.assertFalse(contact_state(.08, .13, .10))
        self.assertTrue(contact_state(.11, .13, .10))
