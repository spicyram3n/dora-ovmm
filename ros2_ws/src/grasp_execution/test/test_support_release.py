"""The initial support exception must never permit shelf/robot collisions."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'core'))
from finish_pick import support_contact, primitive_center
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
import numpy as np


class SupportReleaseTest(unittest.TestCase):
    def test_normalized_collision_object_pose_is_composed(self):
        target = CollisionObject()
        target.pose.orientation.w = 1.
        target.pose.position.x = .65
        target.pose.position.z = .77
        primitive = Pose()
        primitive.position.y = .01
        target.primitive_poses = [primitive]
        np.testing.assert_allclose(primitive_center(target), [.65, .01, .77])

    def setUp(self):
        self.geometry = dict(center_x=.65, center_y=0., radius=.033, min_z=.66, max_z=.886)
        self.contact = SimpleNamespace(contact_body_1='grasp_target', contact_body_2='<octomap>',
                                       depth=.0005, position=SimpleNamespace(x=.65, y=0., z=.66))

    def test_only_shallow_initial_bottom_contact(self):
        self.assertTrue(support_contact(self.contact, .001, self.geometry))
        self.assertFalse(support_contact(self.contact, .006, self.geometry))
        self.contact.depth = .002
        self.assertFalse(support_contact(self.contact, 0., self.geometry))

    def test_robot_contact_is_never_allowed(self):
        self.contact.contact_body_1 = 'hand_l_distal_link'
        self.assertFalse(support_contact(self.contact, 0., self.geometry))

    def test_upper_shelf_and_neighbor_are_never_allowed(self):
        self.contact.position.z = .92
        self.assertFalse(support_contact(self.contact, 0., self.geometry))
        self.contact.position.z = .66
        self.contact.position.x = .75
        self.assertFalse(support_contact(self.contact, 0., self.geometry))


if __name__ == '__main__':
    unittest.main()
