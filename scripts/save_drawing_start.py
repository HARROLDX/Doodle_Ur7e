"""Save a stationary, verified high observation posture. Does not send motion."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
import yaml
from scipy.spatial.transform import Rotation
from doodle_ur7e.core import load_config, paper_transform, check_workspace
from doodle_ur7e.ros import Robot


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default='config/local.yaml')
    p.add_argument('--paper', default='runs/paper.yaml')
    p.add_argument('--output', default='runs/drawing_start.json')
    a = p.parse_args()
    out = Path(a.output)
    if out.exists():
        raise ValueError(f'{out} already exists; keep the accepted start, or use a different --output')
    c = load_config(a.config)
    cal = yaml.safe_load(Path(a.paper).read_text())
    if cal.get('verified') is not True or not all(c['verified'].get(k) is True for k in ['pen_tcp', 'camera_mount', 'workspace', 'scene']):
        raise ValueError('verified local config and paper required')
    if c.get('execution_environment') != 'isaac' and (cal.get('synthetic', False) or cal.get('environment') == 'isaac'):
        raise ValueError('synthetic/simulation paper cannot define a hardware start')
    P = paper_transform(cal, c)
    rclpy.init(args=[])
    r = Robot(c)
    try:
        r.controller_check()
        r.scene()
        state = r.state()
        T = r.fk(state)
        tip = T[:3, 3] + T[:3, :3] @ np.asarray(c['tip_in_link'])
        check_workspace(tip, c)
        if (Rotation.from_quat(c['link_quaternion']).inv() * Rotation.from_matrix(T[:3, :3])).magnitude() > .03:
            raise ValueError('start orientation differs from config')
        if P[:3, 2] @ (tip-P[:3, 3]) < .115:
            raise ValueError('new start must have >=115mm paper clearance for initial 100mm hover trial')
        measured = dict(zip(state.name, state.position))
        record = dict(joint_names=list(c['joint_names']), positions=[measured[n] for n in c['joint_names']],
                      tip_base_m=tip.tolist(), wrist_quaternion_xyzw=Rotation.from_matrix(T[:3, :3]).as_quat().tolist(),
                      captured_unix=time.time(), source='stationary live robot; no motion sent')
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open('x') as stream:
            json.dump(record, stream, indent=2)
        print(f'Saved {out}; no motion sent. Do not replace this with the drawing endpoint.')
    finally:
        r.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, TimeoutError, OSError) as exc:
        raise SystemExit(f'ERROR: {exc}')
