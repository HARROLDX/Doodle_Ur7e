"""Actual Humble DDS services/action with deterministic fake robot, not physical IK."""
import copy
import json
import threading
import time
import os
import signal
import subprocess
import sys
from pathlib import Path
import numpy as np
import pytest
import yaml
rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningScene
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.msg import ControllerState
from control_msgs.action import FollowJointTrajectory
from doodle_ur7e.core import load_config,load_strokes
from doodle_ur7e.ros import Robot,pose
ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.ros


class FakeRobot(Node):
    def __init__(self,c):
        super().__init__('fake_doodle_robot')
        self.c=c; self.scene=PlanningScene(); self.fraction=1.; self.requests=[]; self.executions=0; self.mode="success"; self.cancelled=False
        self.last_pose=pose([0,.5,-.01],c['link_quaternion'])
        self.pub=self.create_publisher(JointState,'/joint_states',10)
        self.create_timer(.02,self.publish)
        self.service_handles=[self.create_service(GetCartesianPath,'/compute_cartesian_path',self.cart),
                       self.create_service(GetPositionFK,'/compute_fk',self.fk),
                       self.create_service(ApplyPlanningScene,'/apply_planning_scene',self.apply),
                       self.create_service(GetPlanningScene,'/get_planning_scene',self.get_scene),
                       self.create_service(ListControllers,'/controller_manager/list_controllers',self.controllers)]
        self.server=ActionServer(self,FollowJointTrajectory,c['action'],self.execute,
                                 callback_group=ReentrantCallbackGroup(), cancel_callback=lambda h: CancelResponse.ACCEPT)

    def publish(self):
        msg=JointState(name=self.c['joint_names'],position=[0.]*6,velocity=[0.]*6)
        msg.header.stamp=self.get_clock().now().to_msg(); self.pub.publish(msg)

    def fk(self,req,res):
        p=PoseStamped(); p.header.frame_id=self.c['base_frame']
        p.pose=copy.deepcopy(self.last_pose if max(req.robot_state.joint_state.position)>.0001
                             else pose([0,.5,-.01],self.c['link_quaternion']))
        res.pose_stamped=[p]; res.fk_link_names=[self.c['ik_link']]; res.error_code.val=1
        return res

    def cart(self,req,res):
        self.requests.append(req); self.last_pose=copy.deepcopy(req.waypoints[-1])
        res.error_code.val=1; res.fraction=self.fraction
        res.solution.joint_trajectory.joint_names=self.c['joint_names']
        res.solution.joint_trajectory.points=[JointTrajectoryPoint(positions=[x]*6) for x in [0.,.001,.002,.003]]
        return res

    def apply(self,req,res): self.scene=copy.deepcopy(req.scene); res.success=True; return res
    def get_scene(self,req,res): res.scene=self.scene; return res
    def controllers(self,req,res):
        res.controller=[ControllerState(name='scaled_joint_trajectory_controller',state='active')]; return res
    def execute(self,handle):
        self.executions+=1
        assert all(len(p.velocities)==6 and not p.accelerations for p in handle.request.trajectory.points)
        if self.mode == 'abort':
            handle.abort(); return FollowJointTrajectory.Result(error_code=-4,error_string='test failure')
        if self.mode == 'wait':
            deadline=time.monotonic()+5
            while not handle.is_cancel_requested and time.monotonic()<deadline: time.sleep(.01)
            if handle.is_cancel_requested:
                self.cancelled=True; handle.canceled(); return FollowJointTrajectory.Result(error_code=0)
            handle.abort(); return FollowJointTrajectory.Result(error_code=-4)
        handle.succeed(); return FollowJointTrajectory.Result(error_code=0)


@pytest.fixture
def system():
    rclpy.init()
    c=load_config(ROOT/'config/lab.yaml')
    fake=FakeRobot(c); ex=MultiThreadedExecutor(num_threads=2); ex.add_node(fake)
    thread=threading.Thread(target=ex.spin,daemon=True); thread.start()
    robot=Robot(c)
    robot.scene(apply=True)
    cal=yaml.safe_load((ROOT/'examples/paper.synthetic.yaml').read_text())
    lines=load_strokes(ROOT/'examples/strokes.json')
    yield robot,fake,c,cal,lines
    robot.destroy_node(); ex.shutdown(); thread.join(timeout=3); fake.destroy_node(); rclpy.shutdown()


def test_plan_no_motion_and_explicit_execution(system,tmp_path):
    robot,fake,c,cal,lines=system
    robot.plan(lines,cal,.01,tmp_path)
    assert fake.executions==0
    req=fake.requests[-1]
    assert req.group_name=='ur_manipulator' and req.link_name=='wrist_3_link'
    assert req.header.frame_id=='base_link' and req.avoid_collisions and req.start_state.is_diff
    assert req.max_step==.002
    assert json.loads((tmp_path/'plan.json').read_text())['executed'] is False
    # Test-only verification flags, NEVER configuration for real hardware.
    c['verified']={k:True for k in c['verified']}; cal['verified']=True; cal['synthetic']=False
    robot.plan(lines,cal,.01,tmp_path,execute=True)
    assert fake.executions==1
    assert json.loads((tmp_path/'plan.json').read_text())['executed'] is True


def test_partial_path_rejected(system,tmp_path):
    robot,fake,c,cal,lines=system; fake.fraction=.99
    with pytest.raises(ValueError,match='incomplete'): robot.plan(lines,cal,.01,tmp_path)
    assert fake.executions==0


def test_unverified_execution_rejected(system,tmp_path):
    robot,fake,c,cal,lines=system
    with pytest.raises(ValueError,match='verification'): robot.plan(lines,cal,.01,tmp_path,True)
    assert not fake.requests and fake.executions==0


def test_wrong_scene_rejected(system):
    robot,fake,_,_,_=system
    fake.scene.world.collision_objects[0].primitives[0].dimensions=[1.,1.,1.]
    with pytest.raises(ValueError,match='geometry differs'): robot.scene()

def test_scene_pose_normalization(system):
    robot,fake,_,_,_=system
    # MoveIt may store an object's translation in object.pose, not primitive_pose.
    obj=fake.scene.world.collision_objects[0]
    obj.pose=copy.deepcopy(obj.primitive_poses[0])
    obj.primitive_poses[0]=pose([0,0,0],[0,0,0,1])
    robot.scene()


def test_synthetic_calibration_blocked(system,tmp_path):
    robot,fake,c,cal,lines=system
    c['verified']={k:True for k in c['verified']}; cal['verified']=True
    with pytest.raises(ValueError,match='non-synthetic'): robot.plan(lines,cal,.01,tmp_path,True)
    assert fake.executions==0


@pytest.mark.parametrize('mode',['abort','wait'])
def test_execution_failure_cancellation(system,tmp_path,mode):
    robot,fake,c,cal,lines=system
    c['verified']={k:True for k in c['verified']}; cal['verified']=True; cal['synthetic']=False
    fake.mode=mode
    if mode=='wait': c['motion']['execution_timeout']=.1
    with pytest.raises((RuntimeError,TimeoutError)):
        robot.plan(lines,cal,.01,tmp_path,True)
    assert fake.executions==1
    if mode=='wait':
        deadline=time.monotonic()+1
        while not fake.cancelled and time.monotonic()<deadline: time.sleep(.01)
        assert fake.cancelled
    assert json.loads((tmp_path/'plan.json').read_text())['executed'] is False


@pytest.mark.parametrize('phase,signum', [
    ('pending', signal.SIGINT), ('pending', signal.SIGTERM),
    ('active', signal.SIGINT), ('acceptance_timeout', None)])
def test_cli_cancels_even_before_acceptance(system,tmp_path,phase,signum):
    robot,fake,c,cal,lines=system
    fake.mode='wait'
    received=threading.Event()
    def accept(req):
        received.set()
        if phase != 'active': time.sleep(.6)
        return GoalResponse.ACCEPT
    fake.server.register_goal_callback(accept)
    c['verified']={k:True for k in c['verified']}; cal['verified']=True; cal['synthetic']=False
    cfg=tmp_path/'config.yaml'; paper=tmp_path/'paper.yaml'
    cfg.write_text(yaml.safe_dump(c)); paper.write_text(yaml.safe_dump(cal))
    code="from doodle_ur7e import ros, cli; "
    if phase=='acceptance_timeout': code+='ros.GOAL_RESPONSE_TIMEOUT=0.1; '
    args=['--config',str(cfg),'draw',str(ROOT/'examples/strokes.json'),'--paper',str(paper),
          '--hover','0.01','--execute','--supervised','--output',str(tmp_path/'plan'),
          '--ros-args','-r','__node:=doodle_cli_signal_check']
    code+='cli.main('+repr(args)+')'
    child=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    try:
        assert received.wait(10), 'CLI never sent its goal'
        if phase=='active':
            deadline=time.monotonic()+3
            while not fake.executions and time.monotonic()<deadline: time.sleep(.01)
            assert fake.executions==1
        if signum is not None: child.send_signal(signum)
        output,_=child.communicate(timeout=12)
        assert child.returncode == (1 if phase=='acceptance_timeout' else 130), output
        assert fake.cancelled, output
        assert 'Goal terminal status confirmed: 5' in output, output
        assert 'CANCEL UNCONFIRMED' not in output, output
        assert not json.loads((tmp_path/'plan'/'plan.json').read_text())['executed']
    finally:
        if child.poll() is None: child.kill();child.wait()


def test_isaac_calibration_rejected_by_hardware(system,tmp_path):
    robot,fake,c,cal,lines=system
    c['verified']={k:True for k in c['verified']}
    cal.update(verified=True,synthetic=False,environment='isaac')
    with pytest.raises(ValueError,match='simulation calibration'):
        robot.plan(lines,cal,0.,tmp_path,True)
    assert fake.executions==0


def test_simulation_clock_and_domain(monkeypatch):
    c=load_config(ROOT/'config/lab.yaml')
    c.update(execution_environment='isaac',use_sim_time=True)
    monkeypatch.setenv('ROS_DOMAIN_ID','0')
    with pytest.raises(ValueError,match='isolated'):
        Robot(c)  # Guard rejects before a ROS node/context is even created.
    monkeypatch.setenv('ROS_DOMAIN_ID','231')
    rclpy.init()
    robot=Robot(c)
    try:
        assert robot.get_parameter('use_sim_time').value is True
    finally:
        robot.destroy_node();rclpy.shutdown()


def test_mount_composes_native_optical_tf_without_reparenting(system,monkeypatch):
    from geometry_msgs.msg import TransformStamped
    from scipy.spatial.transform import Rotation
    from doodle_ur7e import ros
    from doodle_ur7e.core import transform
    robot,_,c,_,_=system
    c['camera']['optical_frame']='camera_color_optical_frame'
    native=TransformStamped()
    native.transform.translation.x=.017
    q=Rotation.from_euler('xyz',[-1.5708,0,-1.5708]).as_quat()
    native.transform.rotation.x,native.transform.rotation.y,native.transform.rotation.z,native.transform.rotation.w=map(float,q)
    monkeypatch.setattr(robot.buffer,'can_transform',lambda *args:True)
    monkeypatch.setattr(robot.buffer,'lookup_transform',lambda parent,child,stamp:native)
    sent=[]
    class Broadcaster:
        def __init__(self,node):pass
        def sendTransform(self,msg):sent.append(msg)
    monkeypatch.setattr(ros,'StaticTransformBroadcaster',Broadcaster)
    def stop(*a,**kw):raise KeyboardInterrupt
    monkeypatch.setattr(ros.rclpy,'spin_once',stop)
    with pytest.raises(KeyboardInterrupt):robot.mount()
    assert sent[0].header.frame_id=='wrist_3_link'
    assert sent[0].child_frame_id=='camera_link'
    np.testing.assert_allclose(ros.matrix(sent[0])@ros.matrix(native),
        transform(c['camera']['mount_position'],c['camera']['mount_quaternion']),atol=1e-12)
