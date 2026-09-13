import unittest
import numpy as np
from core.grasping.pick import merge_target_views,foreground_occlusion


class ViewFusionTests(unittest.TestCase):
    def test_nearer_foreground_is_distinguished_from_background(self):
        mask=np.zeros((50,50),bool);mask[10:40,15:35]=True
        depth=np.full((50,50),2.);depth[mask]=1.
        self.assertEqual(foreground_occlusion(depth,mask),0.)
        depth[40:45,:]=.7
        self.assertGreater(foreground_occlusion(depth,mask),.08)

    def test_complementary_surfaces_restore_height(self):
        theta=np.linspace(-1.4,1.4,40)
        def view(lo,hi):
            p=np.array([[.03*np.cos(t),.03*np.sin(t),z]
                        for z in np.linspace(lo,hi,70) for t in theta])
            return {'points':p}
        result=merge_target_views([view(1.05,1.19),view(1.14,1.275)])
        self.assertAlmostEqual(np.ptp(result['points'][:,2]),.225,places=3)

    def test_different_instance_or_moved_object_is_rejected(self):
        rng=np.random.default_rng(0)
        p=rng.uniform(0,.05,(1000,3))
        with self.assertRaises(RuntimeError):
            merge_target_views([{'points':p},{'points':p+[.15,0,0]}])

if __name__=='__main__':unittest.main()
