"""Contact placement from observed geometry and the calibrated HSRC pad motion.

No object names, mesh assets, or simulator ground truth enter these decisions.
Unresolved geometry or an over-wide contact section fails before robot motion.
"""
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull, cKDTree

GRIPPER_DIR = Path(__file__).resolve().parents[2] / 'docker/graspgenx/x_grippers/hsrc_hand'

# Heights are metres in odom (floor-referenced), not camera depth. This is a
# conservative selection heuristic; MoveIt still checks IK and collisions.
# Cereal boxes use the cuboid policy, based on measured geometry, not a label.
SHAPE_POLICIES = {
    'cylinder': {'approach': 'front'},
    'sphere': {'approach': 'top_down', 'top_down_max_z': .95},
    'cube': {'approach': 'top_down', 'top_down_max_z': .95},
    'cuboid': {'approach': 'front', 'wide_approach': 'top_down',
               'top_down_max_z': .95},
}


def cloud(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 100 or not np.isfinite(points).all():
        raise RuntimeError('Insufficient finite target geometry')
    if np.ptp(points, axis=0).max() > .6:
        raise RuntimeError('Target geometry exceeds the grasp workspace')
    return points


def pad_for_width(width):
    profile = json.loads((GRIPPER_DIR / 'closing_profile.json').read_text())
    gaps = np.array([s['gap'][0] for s in profile])
    if not gaps[0] <= width <= gaps[-1] - .012:
        raise RuntimeError(f'Contact width {width:.3f} m outside calibrated aperture')
    return np.array([np.interp(width, gaps, [s['center'][i] for s in profile]) for i in range(3)])


def calibrated_palm_pose(rotation,contact,width):
    """Place the calibrated contact centre, rather than the palm, at contact.

    Inputs use the canonical gripper frame. The caller converts to the robot
    palm frame exactly once after this width-dependent offset is applied.
    """
    pose=np.eye(4)
    pose[:3,:3]=rotation
    pose[:3,3]=np.asarray(contact)-np.asarray(rotation)@pad_for_width(width)
    return pose


def cylinder(points):
    bottom, top = np.quantile(points[:, 2], [.01, .99])
    side = points[(points[:, 2] > bottom + .2*(top-bottom)) &
                  (points[:, 2] < top - .2*(top-bottom)), :2]
    if len(side) < 100 or top-bottom < .04:
        return None
    origin = side.mean(0)
    xy = side-origin
    a = np.column_stack([2*xy, np.ones(len(xy))])
    if np.linalg.matrix_rank(a) < 3:
        return None
    fit = np.linalg.lstsq(a, np.sum(xy*xy, axis=1), rcond=None)[0]
    radius2 = fit[2] + fit[:2]@fit[:2]
    if radius2 <= 0:
        return None
    radius = np.sqrt(radius2)
    centre = origin+fit[:2]
    residual = np.sqrt(np.mean((np.linalg.norm(side-centre, axis=1)-radius)**2))
    if not .008 <= radius <= .15 or residual > .0015:
        return None
    if np.linalg.norm(((side-centre)/radius).mean(0)) > .95:
        return None  # too little visible arc to infer the hidden centre
    return centre, radius, bottom, top


def top_rectangle(points):
    top = np.quantile(points[:, 2], .98)
    face = points[np.abs(points[:, 2]-top) < .003, :2]
    if len(face) < max(100, .08*len(points)):
        return None
    rect = cv2.minAreaRect(face.astype(np.float32))
    corners = cv2.boxPoints(rect).astype(float)
    edges = np.roll(corners, -1, axis=0)-corners
    lengths = np.linalg.norm(edges, axis=1)
    if lengths.min() < .012 or lengths.max() > .45:
        return None
    area = ConvexHull(face).volume
    if area / (lengths[0]*lengths[1]) < .9:
        return None  # circular/irregular top, rather than a resolved rectangle
    index = int(np.argmin(lengths))
    return corners, edges[index]/lengths[index], lengths[index], top


def sphere(points):
    """Resolve a rounded body from a broad curved patch, tolerating a small dimple."""
    points = cloud(points)
    if len(points) < 100:
        return None
    origin = points.mean(0)
    local = points-origin
    if np.linalg.svd(local, compute_uv=False)[-1]/np.sqrt(len(points)) < .004:
        return None
    keep = np.ones(len(points), dtype=bool)
    for _ in range(3):
        a = np.column_stack([2*local[keep], np.ones(keep.sum())])
        fit = np.linalg.lstsq(a, np.sum(local[keep]**2, axis=1), rcond=None)[0]
        squared = fit[3]+fit[:3]@fit[:3]
        if squared <= 0:
            return None
        radius = np.sqrt(squared); centre = origin+fit[:3]
        error = np.abs(np.linalg.norm(points-centre, axis=1)-radius)
        keep = error <= np.quantile(error, .85)
    if not .015 <= radius <= .055 or np.sqrt(np.mean(error[keep]**2)) > .0025:
        return None
    if np.linalg.norm(((points-centre)/radius).mean(0)) > .85:
        return None
    return centre, radius


def rectangular_candidates(rect, bottom, camera_position):
    """Select a box approach, then return calibrated canonical palm poses."""
    corners, short_axis, short_width, top = rect
    height = top-bottom
    if height < .015:
        raise RuntimeError('Only a flat top is visible; object height is unresolved')
    lengths = np.linalg.norm(np.roll(corners, -1, axis=0)-corners, axis=1)
    dimensions = np.r_[lengths[:2], height]
    shape = 'cube' if dimensions.max()/dimensions.min() < 1.15 else 'cuboid'
    policy = SHAPE_POLICIES[shape]
    centre = corners.mean(0)
    toward = centre-np.asarray(camera_position)[:2]
    if np.linalg.norm(toward) < .01:
        raise RuntimeError('Front approach direction is unresolved')
    toward /= np.linalg.norm(toward)
    screen_horizontal = np.array([-toward[1], toward[0]])
    horizontal_width = np.ptp(corners@screen_horizontal)
    profile = json.loads((GRIPPER_DIR/'closing_profile.json').read_text())
    max_width = profile[-1]['gap'][0]-.012
    mode = policy['approach']
    if horizontal_width > max_width:
        mode = policy.get('wide_approach', 'top_down')
    if top > policy['top_down_max_z']:
        mode = 'front'

    poses, widths = [], []
    axes = [short_axis, np.array([-short_axis[1], short_axis[0]])]
    for axis in axes:
        width = float(np.ptp(corners@axis))
        if mode == 'top_down' and width > short_width*1.1:
            continue
        try:
            pad_for_width(width)
        except RuntimeError:
            continue
        for sign in [1., -1.]:
            if mode == 'top_down':
                closing = np.r_[sign*axis, 0.]
                approach = np.array([0., 0., -1.])
                contact = np.r_[centre, top-min(.025, .35*width, .45*height)]
            else:
                approach = np.r_[sign*np.array([-axis[1], axis[0]]), 0.]
                # Approach only from the observed/front half of the object.
                if approach[:2]@toward < .5:
                    continue
                closing = -np.cross([0., 0., 1.], approach)
                depth = float(np.ptp(corners@approach[:2]))
                penetration = min(.025, .35*width, .45*depth)
                contact = np.r_[centre, (bottom+top)/2]
                contact -= approach*(depth/2-penetration)
            orientation = np.column_stack([closing, np.cross(approach, closing), approach])
            poses.append(calibrated_palm_pose(orientation, contact, width))
            widths.append(width)
            if mode == 'front' and shape == 'cuboid':
                # The opposite wrist roll can clear the forearm from a low
                # table. Recalibrate at each contact, preserving pad placement.
                raised = contact + np.array([0., 0., min(.04, .2*height)])
                flipped = orientation @ np.diag([-1., -1., 1.])
                for rotation, point in [(flipped, contact), (orientation, raised), (flipped, raised)]:
                    poses.append(calibrated_palm_pose(rotation, point, width))
                    widths.append(width)
    if not poses:
        raise RuntimeError(f'No aperture-compatible {shape} {mode} grasp; '
                           f'top z={top:.3f} m, horizontal width={horizontal_width:.3f} m')
    return poses, widths, f'{shape} {mode}'


def contact_candidates(points, palms, camera_position):
    """Return collision envelope, corrected palm poses, widths, and geometry kind.

    Cylinder side grasps retain generated headings. Resolved rectangular tops
    follow SHAPE_POLICIES for width and height. Other shapes retain feasible
    front GraspGenX orientations and use only observed local contact sections.
    """
    points = cloud(points)
    rotation = np.array(json.loads((GRIPPER_DIR/'config.json').read_text())['base_rotation'])
    canonical = np.asarray(palms) @ np.linalg.inv(rotation)
    cyl, rect = cylinder(points), top_rectangle(points)
    results, widths = [], []
    envelope = points
    kind = 'observed front section'
    if cyl is not None:
        centre, radius, bottom, top = cyl
        width = 2*radius
        pad = pad_for_width(width)
        theta = np.linspace(0, 2*np.pi, 64, endpoint=False)
        envelope = np.array([[centre[0]+radius*np.cos(t), centre[1]+radius*np.sin(t), z]
                             for z in [bottom, top] for t in theta])
        kind = 'cylinder'
        for pose in canonical:
            if abs(pose[2, 2]) > .35 or abs(pose[2, 0]) > .35:
                continue
            approach = pose[:3, 2].copy(); approach[2] = 0
            approach /= np.linalg.norm(approach)
            toward = np.r_[centre, (bottom+top)/2]-camera_position
            if approach@toward / np.linalg.norm(toward[:2]) < .5:
                continue
            closing = np.cross([0., 0., 1.], approach)
            # The parallel hand admits the opposite closing direction too.
            # Prefer palm +X upward: the alternative wrist roll puts the
            # forearm across the head camera's view in the HSRC side posture.
            closing *= -1
            corrected = np.eye(4)
            corrected[:3, :3] = np.column_stack([closing, np.cross(approach, closing), approach])
            # Higher side contact reproduced the sustained can hold; keep
            # the adjustment proportional for shorter cylinders and below
            # the upper rim. This uses observed dimensions, not object names.
            contact = np.r_[centre, (bottom+top)/2 + min(.03, .25*(top-bottom))]
            corrected = calibrated_palm_pose(corrected[:3,:3],contact,width)
            if np.linalg.norm(corrected[:3, 3]-pose[:3, 3]) > .10:
                continue
            results.append(corrected@rotation); widths.append(width)
    elif rect is not None:
        corners, short_axis, width, top = rect
        bottom = np.quantile(points[:, 2], .01)
        envelope = np.array([[x, y, z] for z in [bottom, top] for x, y in corners])
        poses, widths, kind = rectangular_candidates(rect, bottom, camera_position)
        results = [pose@rotation for pose in poses]
    elif sphere(points) is not None:
        centre, radius = sphere(points)
        width = 2*radius
        pad_for_width(width)
        envelope = centre + radius*np.array([[x,y,z] for x in [-1.,1.]
                                             for y in [-1.,1.] for z in [-1.,1.]])
        toward = centre[:2]-np.asarray(camera_position)[:2]
        heading = np.arctan2(toward[1], toward[0])
        kind = 'sphere'
        if centre[2]+radius <= SHAPE_POLICIES['sphere']['top_down_max_z']:
            for angle in heading+np.array([0., np.pi/2, np.pi, 3*np.pi/2]):
                closing = np.array([np.cos(angle), np.sin(angle), 0.])
                approach = np.array([0., 0., -1.])
                orientation = np.column_stack([closing, np.cross(approach, closing), approach])
                results.append(calibrated_palm_pose(orientation, centre, width)@rotation)
                widths.append(width)
        for angle in heading+np.array([0., -.26, .26, -.52, .52]):
            approach = np.array([np.cos(angle), np.sin(angle), 0.])
            closing = -np.cross([0.,0.,1.], approach)
            orientation = np.column_stack([closing, np.cross(approach, closing), approach])
            for roll in [np.eye(3), np.diag([-1.,-1.,1.])]:
                results.append(calibrated_palm_pose(orientation@roll, centre, width)@rotation)
                widths.append(width)
    else:
        if np.linalg.svd(points-points.mean(0),compute_uv=False)[-1]/np.sqrt(len(points)) < .003:
            raise RuntimeError('A single flat surface does not constrain a grasp; another view is needed')
        for pose in canonical:
            approach = pose[:3, 2]
            toward = points.mean(0)-camera_position
            if abs(approach[2]) > .35 or approach@toward/np.linalg.norm(toward) < .65:
                continue
            local = (points-pose[:3, 3])@pose[:3, :3]
            section = local[np.abs(local[:,1]-.01276) < .0175]
            if len(section) < 100:
                continue
            lo, hi = np.quantile(section,[.01,.99],axis=0)
            width = hi[0]-lo[0]
            try:
                pad = pad_for_width(width)
            except RuntimeError:
                continue
            # Require resolved depth as well as width. Do not invent a back face.
            if hi[2]-lo[2] < .01:
                continue
            contact = np.array([(lo[0]+hi[0])/2,pad[1],(lo[2]+hi[2])/2])
            corrected = calibrated_palm_pose(pose[:3,:3],pose[:3,3]+pose[:3,:3]@contact,width)
            if np.linalg.norm(corrected[:3,3]-pose[:3,3]) > .06:
                continue
            results.append(corrected@rotation); widths.append(width)
    if not results:
        raise RuntimeError(f'No aperture-compatible {kind} grasp in the observed geometry')
    return envelope, np.asarray(results), np.asarray(widths), kind


def verify_object_lift(before, after, expected):
    """Require actual upward object motion and corresponding surface geometry."""
    before, after = cloud(before), cloud(after)
    expected = np.asarray(expected, dtype=float)
    if expected[2] < .015:
        raise RuntimeError('Test lift was too small to verify')
    # Fingers can hide the lower cylinder after closure. Its resolved axis,
    # radius and upper rim remain comparable without requiring the hidden base.
    first, second = cylinder(before), cylinder(after)
    if first is not None and second is not None:
        centre0, radius0, _, top0 = first
        centre1, radius1, _, top1 = second
        rise = top1-top0
        distances = cKDTree(before).query(after-expected)[0]
        coverage = float(np.mean(distances < .01))
        if (abs(radius1-radius0) <= .003 and
                np.linalg.norm(centre1-centre0-expected[:2]) <= .008 and
                rise >= .6*expected[2] and abs(rise-expected[2]) <= .01 and
                coverage >= .7):
            return {'rise_m': float(rise), 'surface_coverage': coverage}
    # A closed hand can hide the bottom of a sphere. Keep its pre-grasp
    # radius fixed while estimating translation from the remaining surface;
    # changing visible quantiles alone is not evidence of an object lift.
    first, second = sphere(before), sphere(after)
    if first is not None and second is not None:
        centre, radius = first
        _, observed_radius = second
        sample = after[::max(1, len(after)//3000)]
        fit = least_squares(lambda c: np.linalg.norm(sample-c, axis=1)-radius,
                            centre, loss='soft_l1', f_scale=.002, max_nfev=50)
        motion = fit.x-centre
        coverage = float(np.mean(cKDTree(before).query(after-expected)[0] < .01))
        top_rise = np.quantile(after[:,2], .95)-np.quantile(before[:,2], .95)
        if (fit.success and abs(observed_radius-radius) <= min(.005, max(.003, .1*radius))
                and np.linalg.norm(motion[:2]-expected[:2]) <= .008
                and motion[2] >= .6*expected[2] and abs(motion[2]-expected[2]) <= .01
                and top_rise >= .6*expected[2] and abs(top_rise-expected[2]) <= .01
                and coverage >= .7):
            return {'rise_m': float(motion[2]), 'surface_coverage': coverage}
    dz = np.quantile(after[:,2],[.05,.5,.95])-np.quantile(before[:,2],[.05,.5,.95])
    if dz[-1] < .6*expected[2] or np.max(np.abs(dz-expected[2])) > .012:
        raise RuntimeError('Object did not rise with the hand (or became too occluded to verify)')
    shifted = after-expected
    distances = cKDTree(before[::max(1,len(before)//3000)]).query(shifted)[0]
    reverse = cKDTree(shifted[::max(1,len(shifted)//3000)]).query(before)[0]
    if np.mean(distances < .01) < .7 or np.mean(reverse < .012) < .6:
        raise RuntimeError('Lifted target surface does not match the original object')
    return {'rise_m':float(np.median(dz)), 'surface_coverage':float(np.mean(distances < .01))}
