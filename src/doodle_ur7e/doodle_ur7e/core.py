"""ROS-independent geometry and conservative trajectory timing. SI units, xyzw."""
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def array(value, shape, name):
    out = np.asarray(value, dtype=float)
    if out.shape != shape or not np.isfinite(out).all():
        raise ValueError(f'{name}: expected finite {shape}')
    return out


def transform(position, quaternion):
    p = array(position, (3,), 'position')
    q = array(quaternion, (4,), 'quaternion')
    if abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError('quaternion must be unit xyzw')
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(q).as_matrix()
    out[:3, 3] = p
    return out


def load_config(path):
    c = yaml.safe_load(Path(path).read_text())
    for key in ['base_frame', 'ik_link', 'group', 'joint_names', 'tip_in_link',
                'link_quaternion', 'workspace_min', 'workspace_max', 'paper', 'motion']:
        if key not in c:
            raise ValueError(f'missing {key}')
    array(c['tip_in_link'], (3,), 'tip_in_link')
    transform([0, 0, 0], c['link_quaternion'])
    lo = array(c['workspace_min'], (3,), 'workspace_min')
    hi = array(c['workspace_max'], (3,), 'workspace_max')
    if np.any(lo >= hi) or len(set(c['joint_names'])) != 6:
        raise ValueError('invalid workspace or six joint names')
    p, m = c['paper'], c['motion']
    if not 0 < p['width'] <= .5 or not 0 < p['height'] <= .5:
        raise ValueError('paper dimensions in meters required')
    if not 0 < p['margin'] < min(p['width'], p['height']) / 2:
        raise ValueError('invalid paper margin')
    for key, maximum in [('clearance', .1), ('max_step', .005), ('joint_speed', .3),
                         ('joint_acceleration', .5), ('tip_speed', .03),
                         ('max_joint_step', .2), ('max_approach', .25),
                         ('max_duration', 7200), ('execution_timeout', 14400)]:
        if not np.isfinite(m[key]) or not 0 < m[key] <= maximum:
            raise ValueError(f'invalid motion.{key}')
    if m['clearance'] < .005:
        raise ValueError('clearance must be at least 5mm')
    for key in ['world_boxes', 'attached_boxes']:
        if not c.get(key):
            raise ValueError(f'{key} must include measured collision geometry')
        for box in c[key]:
            array(box['center'], (3,), 'box center')
            if np.any(array(box['size'], (3,), 'box size') <= 0):
                raise ValueError('box size must be positive')
    ids = [x['id'] for key in ['world_boxes','attached_boxes'] for x in c[key]]
    if len(ids) != len(set(ids)):
        raise ValueError('collision object ids must be unique')
    return c


def load_strokes(path):
    return validate_strokes(json.loads(Path(path).read_text()))


def validate_strokes(lines):
    if not isinstance(lines, list) or not 0 < len(lines) <= 2000:
        raise ValueError('expected 1..2000 strokes')
    result = []
    for line in lines:
        a = np.asarray(line, dtype=float)
        if a.ndim != 2 or a.shape[1] != 2 or len(a) < 2 or not np.isfinite(a).all():
            raise ValueError('each stroke needs >=2 finite XY points')
        if np.any(a < 0) or np.any(a > 1):
            raise ValueError('stroke coordinates must be normalized [0,1]')
        a = a[np.r_[True, np.any(np.diff(a, axis=0) != 0, axis=1)]]
        if len(a) >= 2:
            result.append(a)
    if not result or sum(map(len, result)) > 20000:
        raise ValueError('empty or excessively large drawing')
    return result


def route(lines):
    """Nearest endpoint routing, allow reversal; no dense O(N^2) cost matrix."""
    remaining = [np.array(x, copy=True) for x in lines]
    out = [remaining.pop(0)]
    while remaining:
        choices = [(np.linalg.norm(x[k] - out[-1][-1]), i, k)
                   for i, x in enumerate(remaining) for k in (0, -1)]
        _, i, k = min(choices)
        line = remaining.pop(i)
        out.append(line if k == 0 else line[::-1])
    return out


def paper_transform(calibration, config):
    if calibration['base_frame'] != config['base_frame']:
        raise ValueError('calibration base frame differs')
    if not np.allclose(calibration['paper_size'], [config['paper']['width'], config['paper']['height']], atol=1e-6):
        raise ValueError('calibration paper size differs')
    T = transform(calibration['position'], calibration['quaternion'])
    if T[2, 2] < np.cos(np.deg2rad(10)):
        raise ValueError('paper normal must face up, table tilt <=10 degrees')
    return T


def waypoints(lines, config, calibration, hover=.0):
    if not np.isfinite(hover) or not 0 <= hover <= .10:
        raise ValueError('hover must be 0..0.10m')
    T = paper_transform(calibration, config)
    p, m = config['paper'], config['motion']
    # Isotropic scale preserves image aspect; paper +y is up, image +y down.
    scale = min(p['width'], p['height']) - 2 * p['margin']
    center = np.array([p['width'], p['height']]) / 2
    local = []
    for line in route(validate_strokes([np.asarray(x).tolist() for x in lines])):
        xy = (line - .5) * [scale, -scale] + center
        local.append([*xy[0], hover + m['clearance']])
        local.extend([[*v, hover] for v in xy])
        local.append([*xy[-1], hover + m['clearance']])
    tips = np.asarray(local) @ T[:3, :3].T + T[:3, 3]
    R = Rotation.from_quat(config['link_quaternion']).as_matrix()
    if np.dot(R[:, 2], T[:3, 2]) > -.95:
        raise ValueError('configured link +Z must point toward paper')
    links = tips - R @ np.asarray(config['tip_in_link'])
    check_workspace(tips, config)
    return tips, links


def check_workspace(points, config):
    points = np.asarray(points)
    if not np.isfinite(points).all() or np.any(points < config['workspace_min']) or np.any(points > config['workspace_max']):
        raise ValueError('pen tip outside configured workspace')


def approach(current_tip, first_tip, config, calibration):
    T = paper_transform(calibration, config)
    local = T[:3, :3].T @ (current_tip - T[:3, 3])
    first = T[:3, :3].T @ (first_tip - T[:3, 3])
    if local[2] < first[2] - .001:
        raise ValueError('start tip too low; manually position above clearance plane')
    if np.linalg.norm(current_tip - first_tip) > config['motion']['max_approach']:
        raise ValueError('start too far away; manually position near drawing')
    # Traverse at current safe height, then descend vertically to first hover.
    above = T[:3, :3] @ np.array([first[0], first[1], local[2]]) + T[:3, 3]
    points = np.array([current_tip, above, first_tip])
    check_workspace(points, config)
    return points


def timing(positions, config, tip_segment_bounds=None):
    q = np.asarray(positions, dtype=float)
    if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or not np.isfinite(q).all():
        raise ValueError('trajectory needs >=2 finite six-joint states')
    dq = np.abs(np.diff(q, axis=0))
    m = config['motion']
    if np.max(dq) > m['max_joint_step']:
        raise ValueError('joint discontinuity; refusing trajectory')
    # Cubic interpolation with zero endpoint velocities: peak |v|=1.5*dq/dt,
    # peak |a|=6*dq/dt^2. No accelerations supplied -> cubic JTC spline.
    # 2m conservative bound on each revolute joint's distance to pen tip for UR7e
    # with <=0.4m tool. Sum Jacobian column bounds controls tip speed throughout.
    tip_bound = 2.0*dq.sum(axis=1)
    if tip_segment_bounds is not None:
        tip_bound = array(tip_segment_bounds, (len(dq),), 'tip segment bounds')
        if np.any(tip_bound < 0):
            raise ValueError('negative tip segment bound')
    dt = np.maximum.reduce([
        1.5 * dq.max(axis=1) / m['joint_speed'],
        np.sqrt(6 * dq.max(axis=1) / m['joint_acceleration']),
        1.5 * tip_bound / m['tip_speed'],
        np.full(len(dq), .05)])
    times = np.r_[.1, .1 + np.cumsum(dt)]
    if times[-1] > m['max_duration']:
        raise ValueError('trajectory exceeds max_duration; simplify drawing')
    return times
