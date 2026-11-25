"""Bounded TCP calibration trial: lift wrist 10mm only, with no drawing descent.

User must confirm axial tool <=290mm and negligible lateral offset. This mode
does not certify TCP or paper calibration and never changes verification flags.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetCartesianPath
from rcl_interfaces.srv import GetParameters
from doodle_ur7e.core import load_config, timing
from doodle_ur7e.kinematics import TipKinematics
from doodle_ur7e.ros import Robot, pose, matrix
from doodle_ur7e.shutdown import stop_signals
from doodle_ur7e.vision import estimate
from rclpy.time import Time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default='config/local.yaml')
    p.add_argument('--output', default='runs/lift_clearance_check.json')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--supervised', action='store_true')
    p.add_argument('--confirmed-max-tcp', type=float, required=True)
    a = p.parse_args()
    if a.confirmed_max_tcp != .290 or (a.execute and not a.supervised):
        raise ValueError('requires confirmed axial TCP <=0.290m and onsite supervision')
    c = load_config(a.config)
    # Cover all 150mm pen placements ending between 240 and 290mm.
    for box in c['attached_boxes']:
        if box['id'] == 'doodle_pen':
            box['center'] = [0., 0., .190]
            box['size'] = [.012, .012, .200]
    c['tip_in_link'] = [0., 0., .290]
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    r = Robot(c)
    try:
        r.controller_check()
        r.scene(apply=True)
        start = r.state()
        T = r.fk(start)
        image, msg, info = r.capture()
        cam_paper, rms = estimate(image, np.array(info.k).reshape(3, 3), np.array(info.d), c['paper'])
        stamp = Time.from_msg(msg.header.stamp)
        r.until(lambda: r.buffer.can_transform(c['base_frame'], msg.header.frame_id, stamp), 3.)
        paper = matrix(r.buffer.lookup_transform(c['base_frame'], msg.header.frame_id, stamp)) @ cam_paper
        normal = paper[:3, 2]
        if normal[2] < np.cos(np.deg2rad(10)) or np.dot(T[:3, 2], normal) > -.95:
            raise ValueError('paper/tool orientation unsuitable')
        quat = Rotation.from_matrix(T[:3, :3]).as_quat()
        req = GetCartesianPath.Request()
        req.header.frame_id = c['base_frame']
        req.start_state.joint_state = start
        req.start_state.is_diff = True
        req.group_name, req.link_name = c['group'], c['ik_link']
        req.waypoints = [pose(T[:3, 3]+[0., 0., .010], quat)]
        req.max_step, req.jump_threshold, req.avoid_collisions = .001, 2., True
        if hasattr(req, 'revolute_jump_threshold'):
            req.revolute_jump_threshold = c['motion']['max_joint_step']
        res = r.call(GetCartesianPath, '/compute_cartesian_path', req)
        if res.error_code.val != 1 or res.fraction < 1.-1e-8:
            raise ValueError('incomplete lift path rejected')
        traj = res.solution.joint_trajectory
        if len(traj.joint_names) != 6 or set(traj.joint_names) != set(c['joint_names']):
            raise ValueError('trajectory joint mismatch')
        q = np.array([v.positions for v in traj.points])
        initial = dict(zip(start.name, start.position))
        if np.max(np.abs(q[0]-[initial[n] for n in traj.joint_names])) > .001:
            raise ValueError('lift path start mismatch')
        minimum = float('inf')
        for source in ['/move_group/get_parameters', '/robot_state_publisher/get_parameters']:
            description = r.call(GetParameters, source, GetParameters.Request(names=['robot_description']))
            model = TipKinematics(description.values[0].string_value, c['base_frame'], c['ik_link'],
                                  list(traj.joint_names), c['tip_in_link'])
            base_tip = model.point(q[0])
            if source.startswith('/move_group') and not np.allclose(model.wrist(q[0]), T, atol=.001, rtol=0):
                raise ValueError('FK disagreement')
            for qa, qb in zip(q[:-1], q[1:]):
                # Bound deviation between samples, not just their values.
                remainder = model.reach*np.abs(qb-qa).sum()**2/(8*32**2)
                for s in np.linspace(0, 1, 33):
                    sampled_q = qa+s*(qb-qa)
                    sample = model.point(sampled_q)
                    angular_error = Rotation.from_matrix(model.wrist(q[0])[:3, :3].T @
                                                         model.wrist(sampled_q)[:3, :3]).magnitude()
                    if angular_error > .002:
                        raise ValueError('lift changes tool orientation excessively')
                    height = float(normal @ (sample-paper[:3, 3]))-remainder
                    minimum = min(minimum, height)
                    if height < .050 or normal @ (sample-base_tip)-remainder < -.0005:
                        raise ValueError('lift loses clearance; rejected')
                    if np.linalg.norm(sample-base_tip)+remainder > .012:
                        raise ValueError('lift exceeds 12mm displacement envelope')
            end = model.wrist(q[-1])
            if np.linalg.norm(end[:3, 3]-T[:3, 3]-[0, 0, .010]) > .002:
                raise ValueError('lift endpoint mismatch')
        times = timing(q, c)  # original conservative stop-at-each-point timing
        for point, sec in zip(traj.points, times):
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(sec*1e9), 10**9)
            point.velocities = [0.]*6
            point.accelerations = []
            point.effort = []
        report = dict(mode='upward_10mm_only', tcp_upper_bound_m=.290,
                      minimum_camera_estimated_clearance_m=minimum, rms_px=rms,
                      duration_s=float(times[-1]), joint_positions=q.tolist(), times=times.tolist(), executed=False)
        out = Path(a.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2))
        print(f'Lift plan complete: {len(q)} points, {times[-1]:.1f}s at 100%; estimated minimum clearance {minimum*1000:.1f}mm.', flush=True)
        if a.execute:
            r.controller_check()
            r.scene()
            now = r.state()
            current = dict(zip(now.name, now.position))
            if max(abs(current[n]-v) for n, v in zip(traj.joint_names, q[0])) > .001:
                raise ValueError('robot moved; replan required')
            r.execute(traj)
            report['executed'] = True
            out.write_text(json.dumps(report, indent=2))
            print('Lift completed. No TCP verification flags changed.')
    finally:
        r.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    with stop_signals():
        main()
