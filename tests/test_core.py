from pathlib import Path
import copy
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from doodle_ur7e.core import load_config, validate_strokes, waypoints, approach, timing, route
ROOT = Path(__file__).resolve().parents[1]

@pytest.fixture
def cfg():
    return load_config(ROOT/'config/lab.yaml')

@pytest.fixture
def cal():
    return yaml.safe_load((ROOT/'examples/paper.synthetic.yaml').read_text())


def test_tip_offset_and_pen_lift(cfg,cal):
    lines = [[[.2,.3],[.3,.4]],[[.7,.7],[.8,.8]]]
    tips, links = waypoints(lines,cfg,cal,0)
    assert np.allclose(links[:,2],tips[:,2]+.22)
    assert np.allclose(tips[[0,3,4,7],2],-.275+.015)
    assert np.allclose(tips[[1,2,5,6],2],-.275)
    assert np.allclose(tips[0,:2],tips[1,:2])
    assert np.allclose(tips[2,:2],tips[3,:2])


def test_tilted_paper_lift_is_normal(cfg,cal):
    cal['quaternion'] = Rotation.from_euler('x',5,degrees=True).as_quat().tolist()
    tips,_ = waypoints([[[.3,.3],[.6,.6]]],cfg,cal)
    normal = Rotation.from_quat(cal['quaternion']).as_matrix()[:,2]
    assert np.allclose(tips[0]-tips[1],normal*.015)


@pytest.mark.parametrize('lines', [[],[[[0,0],[float('nan'),0]]],[[[0,0],[2,0]]],[[[0,0],[0,0]]]])
def test_bad_strokes(lines):
    with pytest.raises(ValueError): validate_strokes(lines)


def test_route_reverse():
    lines = [np.array([[0,0],[.1,0]]),np.array([[.9,0],[.2,0]])]
    assert np.allclose(route(lines)[1][0],[.2,0])


def test_approach_clearance(cfg,cal):
    first = np.array([0,.5,-.25]); current = np.array([.02,.5,-.23])
    p = approach(current,first,cfg,cal)
    assert p[1,2] == current[2]
    with pytest.raises(ValueError): approach(first-[0,0,.01],first,cfg,cal)


def test_timing_cubic_limits_and_jump(cfg):
    q = np.array([[0]*6,[.01,-.02,0,0,0,0],[.03,-.02,.01,0,0,0]])
    t = timing(q,cfg); dt = np.diff(t); dq = np.abs(np.diff(q,axis=0))
    assert np.all(1.5*dq/dt[:,None] <= cfg['motion']['joint_speed']+1e-9)
    assert np.all(6*dq/dt[:,None]**2 <= cfg['motion']['joint_acceleration']+1e-9)
    assert np.all(1.5*2*dq.sum(axis=1)/dt <= cfg['motion']['tip_speed']+1e-9)
    q[1,0] = 3
    with pytest.raises(ValueError): timing(q,cfg)


def test_invalid_paper_and_workspace(cfg,cal):
    cal['quaternion'] = [1,0,0,0]
    with pytest.raises(ValueError): waypoints([[[0,0],[1,1]]],cfg,cal)
    cal['quaternion'] = [0,0,0,1]; cal['position'] = [4,4,4]
    with pytest.raises(ValueError): waypoints([[[0,0],[1,1]]],cfg,cal)


def test_100mm_hover_and_initial_clearance(cfg,cal):
    tips,_=waypoints([[[.4,.4],[.6,.4]]],cfg,cal,.10)
    np.testing.assert_allclose(tips[:,2]-cal['position'][2],[.115,.10,.10,.115])
    approach(tips[0]+[0,0,.005],tips[0],cfg,cal)
    with pytest.raises(ValueError,match='too low'):
        approach(tips[0]-[0,0,.09],tips[0],cfg,cal)
    for invalid in [-.001,.101,float('nan')]:
        with pytest.raises(ValueError,match='hover'):waypoints([[[.4,.4],[.6,.4]]],cfg,cal,invalid)
