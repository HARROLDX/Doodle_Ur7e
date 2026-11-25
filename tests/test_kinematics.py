import numpy as np
import pytest
from doodle_ur7e.kinematics import TipKinematics
from doodle_ur7e.core import timing, load_config
from pathlib import Path


def model():
    joints = ''.join(
        f'<joint name="j{i}" type="revolute"><parent link="l{i}"/>'
        f'<child link="l{i+1}"/><origin xyz="0.1 0 0" rpy="0.1 0.2 0.3"/>'
        f'<axis xyz="0 0 1"/></joint>' for i in range(6))
    return TipKinematics(f'<robot>{joints}</robot>', 'l0', 'l6',
                         [f'j{i}' for i in range(6)], [0.02, 0.01, 0.24])


def test_geometry_bound_covers_interior_tip_velocity():
    m = model()
    rng = np.random.default_rng(42)
    for _ in range(12):
        a = rng.uniform(-3, 3, 6)
        b = a + rng.uniform(-.14, .14, 6)
        bound = m.segment_bounds(np.array([a, b]))[0]
        for s in np.linspace(0, 1, 101):
            eps = 1e-5
            derivative = (m.point(a+(s+eps)*(b-a))-m.point(a+(s-eps)*(b-a)))/(2*eps)
            assert np.linalg.norm(derivative) <= bound+1e-9


def test_geometry_timing_limits_and_zero_motion():
    m = model()
    q = np.array([[0.]*6, [.03, -.04, .02, -.01, .01, -.02], [.03, -.04, .02, -.01, .01, -.02]])
    c = load_config(Path(__file__).resolve().parents[1]/'config/lab.yaml')
    bounds = m.segment_bounds(q)
    dt = np.diff(timing(q, c, bounds))
    dq = np.abs(np.diff(q, axis=0))
    assert np.all(1.5*dq/dt[:, None] <= c['motion']['joint_speed']+1e-9)
    assert np.all(6*dq/dt[:, None]**2 <= c['motion']['joint_acceleration']+1e-9)
    assert np.all(1.5*bounds/dt <= c['motion']['tip_speed']+1e-9)
    assert timing(q, c, bounds)[-1] < timing(q, c)[-1]
    with pytest.raises(ValueError):
        timing(q, c, [-1, 0])
    with pytest.raises(ValueError):
        TipKinematics('<robot/>', 'missing', 'tool', [], [0, 0, 0])
