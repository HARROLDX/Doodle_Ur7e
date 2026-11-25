"""Replan a lifted Cartesian return to the recorded drawing start; no tuck."""
import argparse
import copy
import json
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetCartesianPath
from doodle_ur7e.core import load_config, paper_transform, timing, check_workspace
from doodle_ur7e.ros import Robot, pose
from doodle_ur7e.kinematics import TipKinematics
from doodle_ur7e.shutdown import stop_signals


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default='config/local.yaml')
    p.add_argument('--start', default='runs/drawing_start.json')
    p.add_argument('--paper', default='runs/paper.yaml')
    p.add_argument('--output', default='runs/return_drawing_start.json')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--supervised', action='store_true')
    a = p.parse_args()
    if a.execute and not a.supervised:
        raise ValueError('onsite supervision required')
    c = load_config(a.config)
    saved = json.loads(Path(a.start).read_text())
    calibration = yaml.safe_load(Path(a.paper).read_text())
    if not all(c['verified'].get(k) is True for k in ['pen_tcp', 'camera_mount', 'workspace', 'scene']) or calibration.get('verified') is not True:
        raise ValueError('verified config and paper required')
    if c.get('execution_environment') != 'isaac' and (calibration.get('synthetic', False) or calibration.get('environment') == 'isaac'):
        raise ValueError('synthetic/simulation paper cannot execute on hardware')
    paper = paper_transform(calibration, c)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    r = Robot(c)
    try:
        r.controller_check()
        r.scene()
        state = r.state()
        T = r.fk(state)
        target_state = copy.deepcopy(state)
        target_q = dict(zip(saved['joint_names'], saved['positions']))
        if set(target_q) != set(c['joint_names']):
            raise ValueError('saved start joint names differ')
        target_state.position = [target_q.get(n, v) for n, v in zip(state.name, state.position)]
        goal_T = r.fk(target_state)
        R = goal_T[:3, :3]
        if Rotation.from_matrix(R.T @ T[:3, :3]).magnitude() > .03:
            raise ValueError('current orientation differs from saved start; manual positioning required')
        offset = R @ np.asarray(c['tip_in_link'])
        current_tip = T[:3, 3] + T[:3, :3] @ np.asarray(c['tip_in_link'])
        goal_tip = goal_T[:3, 3] + offset
        current_local = paper[:3, :3].T @ (current_tip-paper[:3, 3])
        goal_local = paper[:3, :3].T @ (goal_tip-paper[:3, 3])
        # Canceled strokes can be close to paper. The first leg only lifts;
        # require a positive 1mm margin and reject on/below-paper starts.
        if current_local[2] < .001 or goal_local[2] < .065 or np.linalg.norm(current_tip-goal_tip) > .25:
            raise ValueError('unsafe/far return start; manually reposition')
        safe_height = max(current_local[2], goal_local[2])+.015
        above_current = current_local.copy(); above_current[2] = safe_height
        above_goal = goal_local.copy(); above_goal[2] = safe_height
        tips = np.array([paper[:3, :3] @ v + paper[:3, 3] for v in [above_current, above_goal, goal_local]])
        check_workspace(np.vstack([current_tip, tips]), c)
        req = GetCartesianPath.Request()
        req.header.frame_id = c['base_frame']
        req.start_state.joint_state = state
        req.start_state.is_diff = True
        req.group_name, req.link_name = c['group'], c['ik_link']
        req.waypoints = [pose(v-offset, Rotation.from_matrix(R).as_quat()) for v in tips]
        req.max_step, req.jump_threshold, req.avoid_collisions = .002, 2., True
        if hasattr(req, 'revolute_jump_threshold'):
            req.revolute_jump_threshold = c['motion']['max_joint_step']
        res = r.call(GetCartesianPath, '/compute_cartesian_path', req)
        if res.error_code.val != 1 or not np.isfinite(res.fraction) or res.fraction < 1.-1e-8:
            raise ValueError('incomplete return rejected')
        traj = res.solution.joint_trajectory
        if len(traj.joint_names) != 6 or set(traj.joint_names) != set(c['joint_names']):
            raise ValueError('return joints differ')
        q = np.asarray([v.positions for v in traj.points])
        measured = dict(zip(state.name, state.position))
        if np.max(np.abs(q[0]-[measured[n] for n in traj.joint_names])) > .001:
            raise ValueError('return start mismatch')
        if np.max(np.abs(q[-1]-[target_q[n] for n in traj.joint_names])) > .01:
            raise ValueError('return IK differs from recorded start')
        bounds = []
        for service in ['/move_group/get_parameters', '/robot_state_publisher/get_parameters']:
            desc = r.call(GetParameters, service, GetParameters.Request(names=['robot_description']))
            model = TipKinematics(desc.values[0].string_value, c['base_frame'], c['ik_link'], list(traj.joint_names), c['tip_in_link'])
            if service.startswith('/move_group') and not np.allclose(model.wrist(q[0]), T, atol=1e-6, rtol=0):
                raise ValueError('return FK mismatch')
            for qa, qb in zip(q[:-1], q[1:]):
                remainder = model.reach*np.abs(qb-qa).sum()**2/(8*32**2)
                samples = np.array([model.point(qa+s*(qb-qa)) for s in np.linspace(0, 1, 33)])
                heights = (samples-paper[:3, 3]) @ paper[:3, 2]
                if heights.min()-remainder < .001:
                    raise ValueError('return interpolation loses positive paper clearance')
                check_workspace(samples, c)
            bounds.append(model.segment_bounds(q))
        times = timing(q, c, np.maximum(*bounds))
        for point, sec in zip(traj.points, times):
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(sec*1e9), 10**9)
            point.velocities = [0.]*6
            point.accelerations = []; point.effort = []
        out = Path(a.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        report = dict(joint_names=list(traj.joint_names), joint_positions=q.tolist(), duration_s=float(times[-1]), executed=False)
        out.write_text(json.dumps(report, indent=2))
        print(f'Return path complete: {len(q)} points, {times[-1]:.1f}s at 100%.', flush=True)
        if a.execute:
            r.controller_check(); r.scene()
            now = r.state(); current = dict(zip(now.name, now.position))
            if max(abs(current[n]-v) for n, v in zip(traj.joint_names, q[0])) > .001:
                raise ValueError('robot moved; replan')
            r.execute(traj)
            report['executed'] = True
            out.write_text(json.dumps(report, indent=2))
            print('Returned to recorded drawing start.')
    finally:
        r.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    try:
        with stop_signals(): main()
    except (ValueError, RuntimeError, TimeoutError, OSError) as exc:
        raise SystemExit(f'ERROR: {exc}')
