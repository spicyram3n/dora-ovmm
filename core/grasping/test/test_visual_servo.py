import unittest
import numpy as np
from core.grasping.visual_servo import image_velocity, hand_image_velocity, project_feature, mask_feature, orientation_velocity, approach_velocity
from scipy.spatial.transform import Rotation


class ImageControlTests(unittest.TestCase):
    def test_final_approach_requires_full_distance(self):
        start=np.eye(4);current=start.copy()
        velocity,finished,progress=approach_velocity(start,current,.08)
        self.assertFalse(finished);self.assertGreater(velocity[2],0)
        self.assertLessEqual(np.linalg.norm(velocity),.0060001)
        current[2,3]=.07
        self.assertFalse(approach_velocity(start,current,.08)[1])
        current[2,3]=.08
        self.assertTrue(approach_velocity(start,current,.08)[1])

    def test_final_approach_rejects_sideways_drift_and_overshoot(self):
        for delta in [[.005,0,.03],[0,0,.084],[0,0,-.005]]:
            current=np.eye(4);current[:3,3]=delta
            with self.assertRaises(RuntimeError):approach_velocity(np.eye(4),current,.08)

    def test_orientation_feedback_reduces_rotation_error(self):
        current=Rotation.from_euler('xyz',[.02,-.03,.01]).as_matrix()
        velocity=orientation_velocity(np.eye(3),current)
        updated=Rotation.from_rotvec(velocity*.1).as_matrix()@current
        self.assertLess(Rotation.from_matrix(updated).magnitude(),Rotation.from_matrix(current).magnitude())
        self.assertLessEqual(np.linalg.norm(velocity),.080001)
    def test_moving_hand_camera_reduces_pixel_error(self):
        k=np.array([[205.,0,320],[0,205.,240],[0,0,1.]])
        # Sideways hand-camera orientation and oblique tool translation axes.
        basis=np.array([[0.,-.8],[1.,0.],[0.,.6]])
        point=np.array([.03,-.02,.25]);desired=np.array([320.,240.])
        uv=project_feature(point,k)
        velocity,error=hand_image_velocity(uv,point[2],desired,k,basis)
        moved=point-basis@velocity*.1
        self.assertLess(np.linalg.norm(project_feature(moved,k)-desired),np.linalg.norm(error))
        self.assertLessEqual(np.linalg.norm(velocity),.0120001)

    def test_hand_mask_rejects_empty_or_clipped_target(self):
        for mask in [np.zeros((30,30),bool),np.ones((30,30),bool)]:
            with self.assertRaises(RuntimeError):mask_feature(mask)

    def test_hand_camera_at_goal_commands_zero(self):
        k=np.array([[205.,0,320],[0,205.,240],[0,0,1.]])
        v,e=hand_image_velocity([330.,250.],.3,[330.,250.],k,np.eye(3)[:,:2])
        np.testing.assert_allclose(v,0,atol=1e-12)

    def test_hand_camera_near_range(self):
        k=np.array([[205.,0,320],[0,205.,240],[0,0,1.]])
        v,e=hand_image_velocity([325.,240.],.124,[320.,240.],k,np.eye(3)[:,:2])
        self.assertGreater(v[0],0)
        for depth in [.01,0.,float('nan')]:
            with self.assertRaises(RuntimeError):
                hand_image_velocity([325.,240.],depth,[320.,240.],k,np.eye(3)[:,:2])

    def test_eye_to_hand_sign_reduces_error(self):
        k=np.array([[500,0,320],[0,500,240],[0,0,1.]])
        point=np.array([.03,-.01,.8]);basis=np.eye(3)[:,:2]
        target=np.array([350,260])
        v,error=image_velocity(point,target,k,basis)
        _,next_error=image_velocity(point+basis@v*.1,target,k,basis)
        self.assertLess(np.linalg.norm(next_error),np.linalg.norm(error))
        self.assertLessEqual(np.linalg.norm(v),.0120001)

    def test_oblique_camera_basis(self):
        k=np.array([[500,0,320],[0,500,240],[0,0,1.]])
        point=np.array([.1,.2,1.]);basis=np.array([[1,0],[0,.8],[0,.6]])
        v,error=image_velocity(point,[380,330],k,basis)
        _,after=image_velocity(point+basis@v*.1,[380,330],k,basis)
        self.assertLess(np.linalg.norm(after),np.linalg.norm(error))

    def test_unobservable_view_rejects_motion(self):
        with self.assertRaises(RuntimeError):image_velocity([0,0,1],[10,10],np.eye(3),np.array([[1,0],[0,0],[0,1]]))

    def test_invalid_depth_rejects_motion(self):
        for z in [-1,0,np.nan]:
            with self.assertRaises(RuntimeError):image_velocity([0,0,z],[1,1],np.eye(3),np.eye(3)[:,:2])

if __name__=='__main__':unittest.main()
