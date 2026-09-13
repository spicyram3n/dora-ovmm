import unittest
import numpy as np
from core.grasping.contact_geometry import cylinder, sphere, contact_candidates, pad_for_width, verify_object_lift, calibrated_palm_pose, SHAPE_POLICIES
from scipy.spatial.transform import Rotation


def can(radius=.033, centre=(.5,0), bottom=.7, height=.22):
    theta=np.linspace(np.pi/2,3*np.pi/2,70)
    return np.array([[centre[0]+radius*np.cos(t),centre[1]+radius*np.sin(t),z]
                     for z in np.linspace(bottom,bottom+height,40) for t in theta])


def box(width=.05,length=.08, bottom=.7, height=.1):
    return np.vstack([np.array([[x,y,bottom+height] for x in np.linspace(-width/2,width/2,30)
                               for y in np.linspace(-length/2,length/2,30)]),
                      np.array([[x,-length/2,z] for x in np.linspace(-width/2,width/2,30)
                                for z in np.linspace(bottom,bottom+height,30)])])


class GeometryTests(unittest.TestCase):
    def test_partial_sphere_recovers_body_and_level_grasps(self):
        centre=np.array([.5,0.,.75]);radius=.045
        points=np.array([centre+radius*np.array([np.sin(p)*np.cos(t),np.sin(p)*np.sin(t),np.cos(p)])
                         for p in np.linspace(.25,2.4,40) for t in np.linspace(1.7,4.5,40)])
        actual,r=sphere(points)
        np.testing.assert_allclose(actual,centre,atol=1e-6)
        self.assertAlmostEqual(r,radius,places=6)
        _,grasps,widths,kind=contact_candidates(points,np.empty((0,4,4)),[0,0,1.])
        self.assertEqual(kind,'sphere')
        self.assertGreater(len(grasps),1)
        np.testing.assert_allclose(grasps[:4,2,2],-1,atol=1e-6)
        np.testing.assert_allclose(grasps[4:,2,2],0,atol=1e-6)
        canonical_from_palm=np.array([[0,1,0],[-1,0,0],[0,0,1]])
        for pose,width in zip(grasps,widths):
            contact=pose[:3,3]+pose[:3,:3]@canonical_from_palm.T@pad_for_width(width)
            np.testing.assert_allclose(contact[:2],centre[:2],atol=1e-6)
            self.assertAlmostEqual(contact[2],centre[2],places=6)
            self.assertAlmostEqual((width/2)**2+(contact[2]-centre[2])**2,radius**2,places=8)

    def test_flat_patch_does_not_invent_a_sphere(self):
        points=np.array([[x,y,.5] for x in np.linspace(0,.05,20) for y in np.linspace(0,.05,20)])
        self.assertIsNone(sphere(points))

    def test_calibration_varies_with_width_and_places_pads_once(self):
        contact=np.array([.5,.1,.8])
        rotation=Rotation.from_euler('xyz',[.3,-.7,1.2]).as_matrix()
        positions=[]
        for width in [.03,.05,.066,.09]:
            pose=calibrated_palm_pose(rotation,contact,width)
            np.testing.assert_allclose(pose[:3,3]+rotation@pad_for_width(width),contact,atol=1e-12)
            positions.append(pose[:3,3])
        self.assertGreater(np.linalg.norm(positions[0]-positions[-1]),.005)

    def test_partial_cylinder_recovers_hidden_axis(self):
        centre,radius,_,_=cylinder(can())
        np.testing.assert_allclose(centre,[.5,0],atol=1e-6)
        self.assertAlmostEqual(radius,.033,places=6)

    def test_cylinder_pad_center_meets_axis(self):
        # Palm +Y closes across world Y, +Z approaches world X.
        palm=np.eye(4);palm[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]];palm[:3,3]=[.39,0,.81]
        envelope,poses,widths,kind=contact_candidates(can(),[palm],[0,0,1.])
        self.assertEqual(kind,'cylinder')
        self.assertGreater(len(poses),0)
        canonical_from_palm=np.array([[0,1,0],[-1,0,0],[0,0,1]])
        actual=poses[0,:3,3]+poses[0,:3,:3]@canonical_from_palm.T@pad_for_width(widths[0])
        np.testing.assert_allclose(actual[:2],[.5,0],atol=.003)
        self.assertGreater(actual[2], .81)
        self.assertLess(actual[2], .92-.03)  # contact stays below the upper rim

    def test_rotated_box_closes_across_short_width(self):
        cloud=box(.05,.20);a=.63;r=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
        _,poses,widths,kind=contact_candidates(cloud@r.T,[np.eye(4)],[0,-.7,1.2])
        self.assertEqual(kind,'cuboid top_down')
        np.testing.assert_allclose(widths,.05,atol=.001)
        np.testing.assert_allclose(poses[:,:3,2],np.tile([0,0,-1],(len(poses),1)),atol=1e-6)

    def test_square_has_both_axes(self):
        _,poses,_,kind=contact_candidates(box(.05,.05,height=.05),[np.eye(4)],[0,-.7,1.2])
        self.assertEqual(kind,'cube top_down')
        self.assertEqual(len(poses),4)

    def test_high_cube_uses_front_only(self):
        _,poses,widths,kind=contact_candidates(box(.057,.057,bottom=1.05,height=.057),
                                             [np.eye(4)],[0,-.7,1.2])
        self.assertEqual(kind,'cube front')
        np.testing.assert_allclose(poses[:,2,2],0,atol=1e-6)
        np.testing.assert_allclose(widths,.057,atol=.001)
        self.assertTrue(np.all(poses[:,1,2]>.9))

    def test_narrow_cuboid_prefers_front(self):
        _,poses,_,kind=contact_candidates(box(),[np.eye(4)],[0,-.7,1.2])
        self.assertEqual(kind,'cuboid front')
        np.testing.assert_allclose(poses[:,2,2],0,atol=1e-6)

    def test_wide_cereal_box_uses_short_edge_from_above(self):
        _,poses,widths,kind=contact_candidates(box(.20,.05,bottom=.4,height=.25),
                                             [np.eye(4)],[0,-.7,1.2])
        self.assertEqual(kind,'cuboid top_down')
        np.testing.assert_allclose(widths,.05,atol=.001)
        np.testing.assert_allclose(poses[:,2,2],-1,atol=1e-6)

    def test_high_wide_box_does_not_force_oversize_front_grasp(self):
        with self.assertRaisesRegex(RuntimeError,'cuboid front'):
            contact_candidates(box(.20,.05,bottom=1.,height=.25),
                               [np.eye(4)],[0,-.7,1.2])

    def test_high_box_front_can_use_visible_short_edge(self):
        _,poses,widths,kind=contact_candidates(box(.20,.05,bottom=1.,height=.25),
                                             [np.eye(4)],[-.7,-.7,1.2])
        self.assertEqual(kind,'cuboid front')
        np.testing.assert_allclose(widths,.05,atol=.001)
        np.testing.assert_allclose(poses[:,2,2],0,atol=1e-6)

    def test_box_too_wide_in_both_axes_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'aperture'):
            contact_candidates(box(.20,.25,bottom=.4,height=.15),
                               [np.eye(4)],[0,-.7,1.2])

    def test_top_height_threshold_includes_object_height(self):
        z=SHAPE_POLICIES['cube']['top_down_max_z']
        for top,expected in [(z-.001,'cube top_down'),(z+.001,'cube front')]:
            _,_,_,kind=contact_candidates(box(.05,.05,bottom=top-.05,height=.05),
                                         [np.eye(4)],[0,-.7,1.2])
            self.assertEqual(kind,expected)

    def test_wide_object_fails_before_motion(self):
        with self.assertRaisesRegex(RuntimeError,'aperture'):
            contact_candidates(can(.08),[np.eye(4)],[0,0,1.])

    def test_unresolved_plane_fails(self):
        p=box()[:900]
        with self.assertRaises(RuntimeError):
            contact_candidates(p,[np.eye(4)],[0,-.7,1.2])

    def test_physical_lift_required_even_if_hand_gap_passed(self):
        p=can()
        with self.assertRaises(RuntimeError): verify_object_lift(p,p,[0,0,.03])
        result=verify_object_lift(p,p+[0,0,.03],[0,0,.03])
        self.assertAlmostEqual(result['rise_m'],.03)

    def test_toppled_or_different_target_rejected(self):
        with self.assertRaises(RuntimeError): verify_object_lift(can(),box(),[0,0,.03])

    def test_lifted_can_with_lower_half_occluded(self):
        before=can();visible=before[before[:,2]>.81]
        result=verify_object_lift(before,visible+[0,0,.03],[0,0,.03])
        self.assertAlmostEqual(result['rise_m'],.03,delta=.006)

    def test_occlusion_alone_cannot_pass_lift(self):
        before=can();visible=before[before[:,2]>.81]
        with self.assertRaises(RuntimeError):
            verify_object_lift(before,visible,[0,0,.03])

    def test_sphere_lift_with_occluded_lower_surface(self):
        centre=np.array([.5,0.,.75])
        before=np.array([centre+.045*np.array([np.sin(p)*np.cos(t),np.sin(p)*np.sin(t),np.cos(p)])
                         for p in np.linspace(.1,2.5,50) for t in np.linspace(1.6,4.6,50)])
        visible=before[before[:,2]>.75]
        evidence=verify_object_lift(before,visible+[0,0,.03],[0,0,.03])
        self.assertAlmostEqual(evidence['rise_m'],.03,delta=.002)
        for after in [visible,visible+[.03,0,.03],centre+1.4*(visible-centre)+[0,0,.03]]:
            with self.assertRaises(RuntimeError):
                verify_object_lift(before,after,[0,0,.03])

    def test_different_radius_or_lateral_slip_cannot_pass(self):
        for after in [can(.045)+[0,0,.03],can()+[.03,0,.03]]:
            with self.assertRaises(RuntimeError):
                verify_object_lift(can(),after,[0,0,.03])


if __name__=='__main__':unittest.main()
