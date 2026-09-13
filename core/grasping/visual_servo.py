"""Bounded eye-in-hand IBVS using the hand RGB camera and MoveIt Servo.

Head RGB-D supplies the initial 3-D target feature and grasp geometry. Hand
segmentation supplies live pixel error; projected target depth scales its
interaction matrix. A moving camera requires the opposite translation sign
from eye-to-hand control. MTC and Servo never command together.
"""
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ChangeDriftDimensions
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from core.perception.camera_ros2 import grab_hand_rgb
from core.perception import sam3_client, pointcloud
from core.utils.transforms import matrix_from_transform


def mask_feature(mask):
    v,u=np.nonzero(mask)
    if len(u)<100:
        raise RuntimeError('Too few hand-camera target pixels')
    if u.min()==0 or v.min()==0 or u.max()==mask.shape[1]-1 or v.max()==mask.shape[0]-1:
        raise RuntimeError('Target is clipped by the hand camera image boundary')
    return np.array([u.mean(),v.mean()]),len(u)


def project_feature(point,k):
    point=np.asarray(point,float)
    if not np.isfinite(point).all() or point[2]<.05:
        raise RuntimeError('Invalid hand-camera feature depth')
    pixel=k@point
    return pixel[:2]/pixel[2]


def hand_image_velocity(uv,depth,desired_uv,k,basis_camera):
    """Translate the camera so the observed image feature approaches its goal."""
    point=np.array([(uv[0]-k[0,2])/k[0,0],(uv[1]-k[1,2])/k[1,1],1.])*depth
    velocity,error=image_velocity(point,desired_uv,k,basis_camera,min_depth=.05)
    return -velocity,error


def orientation_velocity(desired,current):
    error=Rotation.from_matrix(desired@current.T).as_rotvec()
    velocity=1.5*error
    velocity*=min(1.,.08/max(np.linalg.norm(velocity),1e-12))
    return velocity


def approach_velocity(start,current,distance):
    """Bounded Cartesian final approach; never accept a short or skewed reach."""
    axis=start[:3,2]
    delta=current[:3,3]-start[:3,3]
    progress=float(delta@axis)
    lateral=delta-progress*axis
    angle=Rotation.from_matrix(start[:3,:3].T@current[:3,:3]).magnitude()
    if progress < -.004 or progress > distance+.003:
        raise RuntimeError('Servo approach left its axial travel bounds')
    if np.linalg.norm(lateral)>.004 or angle>.04:
        raise RuntimeError('Servo approach left its alignment corridor')
    remaining=distance-progress
    finished=abs(remaining)<.002 and np.linalg.norm(lateral)<.002 and angle<.02
    velocity=axis*np.clip(.8*remaining,0.,.006)-.8*lateral
    velocity*=min(1.,.006/max(np.linalg.norm(velocity),1e-12))
    return velocity,finished,progress


def feature(depth, k, mask):
    valid = pointcloud.object_depth_mask(depth, mask)
    v, u = np.nonzero(valid)
    uv = np.array([u.mean(), v.mean()])
    near = (u-uv[0])**2+(v-uv[1])**2 < 25
    z = np.median(depth[v[near],u[near]]) if near.any() else np.median(depth[valid])
    xyz = np.array([(uv[0]-k[0,2])*z/k[0,0],(uv[1]-k[1,2])*z/k[1,1],z])
    return uv, xyz, int(valid.sum())


def image_velocity(tool_point_camera, target_uv, k, basis_camera, gain=.6, max_speed=.012, min_depth=.15):
    """Positive robot-point image Jacobian for a stationary eye-to-hand camera."""
    x,y,z = np.asarray(tool_point_camera,float)
    if not np.isfinite([x,y,z]).all() or z < min_depth:
        raise RuntimeError('Control feature is behind/too close to the camera')
    predicted = np.array([k[0,0]*x/z+k[0,2],k[1,1]*y/z+k[1,2]])
    error = np.asarray(target_uv)-predicted
    interaction = np.array([[k[0,0]/z,0,-k[0,0]*x/z**2],
                            [0,k[1,1]/z,-k[1,1]*y/z**2]])
    jacobian = interaction@basis_camera
    if not np.isfinite(jacobian).all() or np.linalg.cond(jacobian) > 50:
        raise RuntimeError('Image Jacobian cannot resolve transverse alignment from this view')
    velocity = gain*np.linalg.solve(jacobian,error)
    velocity *= min(1.,max_speed/max(np.linalg.norm(velocity),1e-12))
    return velocity, error


class VisualServo:
    def __init__(self,node):
        self.node = node
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,node)
        self.publisher = node.create_publisher(TwistStamped,'/grasp_servo/delta_twist_cmds',10)
        self.status = None
        self.status_time = 0.
        self.subscription = node.create_subscription(Int8,'/grasp_servo/status',self._status,10)

    def _status(self,message):
        self.status = message.data
        self.status_time = time.monotonic()

    def transform(self,link='hand_palm_link',stamp=None):
        stamp = Time() if stamp is None else stamp
        deadline = time.monotonic()+3
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.02)
            if self.buffer.can_transform('odom',link,stamp):
                transform = self.buffer.lookup_transform('odom',link,stamp)
                age = (self.node.get_clock().now()-Time.from_msg(transform.header.stamp)).nanoseconds/1e9
                if age > .5:
                    continue
                return matrix_from_transform(transform.transform)
        raise RuntimeError(f'No fresh odom-to-{link} transform')

    def service(self,name,kind=Trigger,request=None):
        client=self.node.create_client(kind,'/grasp_servo/'+name)
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError(f'MoveIt Servo {name} unavailable')
            future=client.call_async(request or kind.Request())
            rclpy.spin_until_future_complete(self.node,future,timeout_sec=5.)
            if not future.done() or not future.result().success:
                raise RuntimeError(f'MoveIt Servo {name} failed')
        finally:
            self.node.destroy_client(client)

    def command(self,linear,angular=None):
        msg=TwistStamped()
        msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.header.frame_id='odom'
        msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z=map(float,linear)
        if angular is not None:
            msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z=map(float,angular)
        self.publisher.publish(msg)

    def stop(self):
        # Flush zero commands before pausing; do not leave a position trajectory
        # or residual smoothing velocity active during MTC's next action.
        end=time.monotonic()+.3
        while time.monotonic()<end:
            self.command(np.zeros(3));rclpy.spin_once(self.node,timeout_sec=.02)
        self.service('pause_servo')
        first=self.transform()
        end=time.monotonic()+3.
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=.1)
            second=self.transform()
            if np.linalg.norm(first[:3,3]-second[:3,3]) < .0005:
                return
            first=second
        raise RuntimeError('Arm did not settle after visual servoing')

    def align(self,prompt,pregrasp,reference_feature,reference_area):
        start=self.transform();base=self.transform('base_footprint')
        desired_uv=None; hand_area=None
        basis=pregrasp[:3,:2]  # centring only; preserve axial pregrasp clearance
        history=[];converged=0
        try:
            self.service('start_servo')
            # Keep orientation regulated: free rotational drift can reduce
            # image error by rotating the camera instead of aligning the pads.
            self.service('change_drift_dimensions',ChangeDriftDimensions,
                         ChangeDriftDimensions.Request())
            self.service('unpause_servo')
            deadline=time.monotonic()+180.
            while time.monotonic()<deadline:
                self.command(np.zeros(3))
                rgb,k,odom_camera,image_hand=grab_hand_rgb(timeout=8.)
                captured=time.monotonic()
                mask,_=sam3_client.detect(rgb,prompt,timeout=5.)
                uv,area=mask_feature(mask)
                camera_odom=np.linalg.inv(odom_camera)
                target_camera=(camera_odom@np.r_[reference_feature,1.])[:3]
                if target_camera[2] < .05:
                    raise RuntimeError('Target too close to hand camera')
                observed=np.array([(uv[0]-k[0,2])/k[0,0],
                                   (uv[1]-k[1,2])/k[1,1],1.])*target_camera[2]
                if desired_uv is None:
                    hand_camera=np.linalg.inv(image_hand)@odom_camera
                    desired_point=(np.linalg.inv(pregrasp@hand_camera)@np.r_[reference_feature,1.])[:3]
                    desired_uv=project_feature(desired_point,k)
                    if not (0<=desired_uv[0]<rgb.shape[1] and 0<=desired_uv[1]<rgb.shape[0]):
                        raise RuntimeError('Planned target feature is outside the hand camera view')
                    hand_area=area
                if time.monotonic()-captured > 1.:
                    raise RuntimeError('Visual observation became stale during detection')
                if not .6 <= area/hand_area <= 1.5:
                    raise RuntimeError('Target became occluded or changed identity during IBVS')
                target_world=odom_camera[:3,:3]@observed+odom_camera[:3,3]
                if np.linalg.norm(target_world-reference_feature) > .035:
                    raise RuntimeError('Target association moved beyond the bounded alignment region')
                hand=self.transform();current_base=self.transform('base_footprint')
                if np.linalg.norm(current_base[:3,3]-base[:3,3]) > .003:
                    raise RuntimeError('Base moved during arm-only visual servoing')
                if Rotation.from_matrix(base[:3,:3].T@current_base[:3,:3]).magnitude() > .01:
                    raise RuntimeError('Base rotated during arm-only visual servoing')
                if np.linalg.norm(hand[:3,3]-pregrasp[:3,3]) > .035:
                    raise RuntimeError('IBVS correction exceeded 35 mm')
                if Rotation.from_matrix(pregrasp[:3,:3].T@hand[:3,:3]).magnitude() > .06:
                    raise RuntimeError('Arm orientation drift exceeded the IBVS bound')
                lateral,error=hand_image_velocity(uv,target_camera[2],desired_uv,k,camera_odom[:3,:3]@basis)
                angular=orientation_velocity(pregrasp[:3,:3],hand[:3,:3])
                norm=float(np.linalg.norm(error));history.append(norm)
                print(f'[IBVS] image error {norm:.2f} px',flush=True)
                converged=converged+1 if norm < 2. else 0
                if converged >= 3:
                    return history
                if len(history)>8 and min(history[-4:]) > min(history[:-4])+2.:
                    raise RuntimeError('IBVS image error is increasing')
                # A short velocity burst on fresh visual evidence. Always stop
                # while waiting for the next RGB/SAM3 result (including exceptions).
                until=time.monotonic()+.15
                while time.monotonic()<until:
                    rclpy.spin_once(self.node,timeout_sec=.02)
                    if self.status in (-1,2,4,5) or time.monotonic()-self.status_time > .5:
                        raise RuntimeError(f'MoveIt Servo halted or status stale ({self.status})')
                    self.command(basis@lateral if norm>=2. else np.zeros(3),angular)
                self.command(np.zeros(3))
            raise RuntimeError('IBVS did not converge before timeout')
        finally:
            self.stop()

    def approach(self,prompt,reference_feature,distance=.08,start_pose=None):
        """Servo owns the final straight reach; MTC sends no trajectory here.

        TF closes the distance/orientation loop. Hand-camera segmentation
        checks that the target remains on its predicted image ray. Its outline
        may leave the image during the close approach, so do not use a clipped
        silhouette centroid as an IBVS translation error.
        """
        if not 0.<distance<=.08:
            raise ValueError('Final approach must be within 80 mm')
        start=self.transform() if start_pose is None else np.asarray(start_pose,float)
        approach_velocity(start,self.transform(),distance)
        base=self.transform('base_footprint')
        history=[];settled=0;initial_area=None;best=0.;last_progress=time.monotonic()
        try:
            self.service('start_servo')
            self.service('change_drift_dimensions',ChangeDriftDimensions,ChangeDriftDimensions.Request())
            self.service('unpause_servo')
            deadline=time.monotonic()+300.
            while time.monotonic()<deadline:
                self.command(np.zeros(3))
                rgb,k,camera,_=grab_hand_rgb(timeout=8.)
                captured=time.monotonic()
                mask,_=sam3_client.detect(rgb,prompt,timeout=5.)
                if time.monotonic()-captured>1.:
                    raise RuntimeError('Approach target observation became stale')
                point=(np.linalg.inv(camera)@np.r_[reference_feature,1.])[:3]
                pixel=project_feature(point,k)
                u,v=np.rint(pixel).astype(int)
                if not (0<=u<rgb.shape[1] and 0<=v<rgb.shape[0]):
                    raise RuntimeError('Target reference left the hand-camera view')
                if mask.sum()<100 or not cv2.dilate(mask.astype(np.uint8),np.ones((5,5),np.uint8))[v,u]:
                    raise RuntimeError('Target detection no longer contains the expected approach feature')
                if initial_area is None:initial_area=int(mask.sum())
                if mask.sum()<.6*initial_area:
                    raise RuntimeError('Approach target became substantially occluded')
                hand=self.transform();current_base=self.transform('base_footprint')
                if np.linalg.norm(current_base[:3,3]-base[:3,3])>.003 or Rotation.from_matrix(base[:3,:3].T@current_base[:3,:3]).magnitude()>.01:
                    raise RuntimeError('Base moved during Servo-only approach')
                velocity,finished,progress=approach_velocity(start,hand,distance)
                history.append(progress)
                print(f'[SERVO APPROACH] {progress*1000:.1f}/{distance*1000:.1f} mm',flush=True)
                settled=settled+1 if finished else 0
                if settled>=3:break
                if progress>best+.0005:
                    best=progress;last_progress=time.monotonic()
                if not finished and time.monotonic()-last_progress>20.:
                    raise RuntimeError('Servo approach made no measurable progress')
                angular=orientation_velocity(start[:3,:3],hand[:3,:3])
                # Sustain commands long enough for Servo's smoothing filter
                # while supervising TF and Servo status throughout the burst.
                until=time.monotonic()+1.
                while time.monotonic()<until:
                    rclpy.spin_once(self.node,timeout_sec=.02)
                    if self.status in (-1,2,4,5) or time.monotonic()-self.status_time>.5:
                        raise RuntimeError(f'Servo approach halted or status stale ({self.status})')
                    hand=self.transform()
                    velocity,finished,_=approach_velocity(start,hand,distance)
                    angular=orientation_velocity(start[:3,:3],hand[:3,:3])
                    self.command(np.zeros(3) if finished else velocity,angular)
                self.command(np.zeros(3))
            else:
                raise RuntimeError('Servo approach did not reach its endpoint before timeout')
        finally:
            self.stop()
        # Check the settled endpoint, not just the last sample before stopping.
        _,finished,_=approach_velocity(start,self.transform(),distance)
        if not finished:
            raise RuntimeError('Servo stopped short of the final endpoint; refusing closure')
        return history
