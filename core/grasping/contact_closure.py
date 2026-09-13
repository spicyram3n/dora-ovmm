"""Bounded position closure using both passive finger springs as contact feedback."""
import math
import time
import json
import os
from pathlib import Path


def wait_for_hold(read_contact, clock, seconds=5., timeout=90.):
    """Require uninterrupted bilateral contact for a simulated-time dwell.

    This only establishes contact persistence; the caller must also verify
    that the object stayed elevated using a fresh camera observation.
    """
    started = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        left, right = read_contact()
        if not contact_state(left, right, .06):
            raise RuntimeError('Object slipped: bilateral contact lost after lift')
        now = clock()
        if started is None:
            started = now
        if now < started:
            raise RuntimeError('Simulation clock reset during hold verification')
        if now - started >= seconds:
            return
        time.sleep(.2)
    raise RuntimeError('Timed out waiting for sustained grasp contact')


def contact_state(left, right, threshold):
    if not all(math.isfinite(x) for x in (left, right, threshold)):
        raise ValueError('Non-finite finger feedback')
    if not .06 <= threshold <= .18:
        raise ValueError('Contact threshold outside calibrated range')
    if max(left, right) > .20:
        raise RuntimeError('Excessive or asymmetric finger contact; closure stopped')
    return min(left, right) > threshold


def close(node, threshold=.06):
    # Local import keeps the ROS action helpers shared with the pick entrypoint.
    from core.grasping import pick as p
    motor = p.joint_positions(node)['hand_motor_joint']
    seen_contact = False
    samples = []
    diagnostics = os.environ.get('HSR_GRASP_DIAGNOSTICS')
    for _ in range(100):
        q = p.joint_positions(node)
        left, right = (q[f'hand_{s}_spring_proximal_joint'] for s in 'lr')
        if diagnostics:
            samples.append(dict(time=time.time(), motor=q['hand_motor_joint'], left=left, right=right))
            Path(diagnostics).mkdir(parents=True, exist_ok=True)
            (Path(diagnostics)/'closure_feedback.json').write_text(json.dumps(samples, indent=2))
        if seen_contact and min(left, right) < .02:
            raise RuntimeError('Object contact lost during closure; refusing further squeezing')
        seen_contact |= min(left, right) > .06
        if contact_state(left, right, threshold):
            for _ in range(3):
                time.sleep(.2)
                held = p.joint_positions(node)
                l, r = (held[f'hand_{s}_spring_proximal_joint'] for s in 'lr')
                if not contact_state(l, r, .06):
                    raise RuntimeError('Bilateral contact was lost during position hold')
            print(f'[CONTACT] position hold, springs {left:.3f}, {right:.3f}', flush=True)
            return {'left': left, 'right': right, 'motor': q['hand_motor_joint']}
        motor -= .005 if min(left, right) > .06 else .02
        if motor < .10:
            raise RuntimeError('Minimum closure reached without bilateral contact')
        goal = p.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['hand_motor_joint']
        goal.trajectory.points = [p.JointTrajectoryPoint(
            positions=[float(motor)], time_from_start=p.Duration(nanosec=500000000))]
        result = p.run_action(node, p.FollowJointTrajectory,
                             '/gripper_controller/follow_joint_trajectory', goal, 10.)
        if result.result.error_code != 0:
            raise RuntimeError('Finger position step failed: '+result.result.error_string)
    raise RuntimeError('Closure exhausted without bilateral contact')
