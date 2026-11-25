#!/usr/bin/env python3
"""Exercise the image CLI through actual UR7e MoveIt, with offline joint state.

No hardware, controller or action server is started. Test normal planning and a
blocked --execute request that must fail before reaching the action interface.
"""
import json, os, signal, subprocess, threading, time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from doodle_ur7e.core import load_config
from doodle_ur7e.ros import Robot

ROOT=Path(__file__).resolve().parents[1];A=ROOT/'runs/isaac/assets'
OUT=ROOT/'runs/baymax_moveit';OUT.mkdir(parents=True,exist_ok=True)
if os.environ.get('ROS_DOMAIN_ID')!='231' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise RuntimeError('isolated domain 231 required')
c=load_config(ROOT/'config/isaac.yaml');c['use_sim_time']=False
cfg=OUT/'config.yaml';cfg.write_text(yaml.safe_dump(c))
log=(OUT/'moveit.log').open('w');children=[]
for package,executable,params in [('robot_state_publisher','robot_state_publisher','robot_parameters.yaml'),('moveit_ros_move_group','move_group','moveit_parameters.yaml')]:
    children.append(subprocess.Popen(['ros2','run',package,executable,'--ros-args','--params-file',str(A/params),'-p','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
rclpy.init();robot=Robot(c);publisher=Node('image_test_joint_state')
pub=publisher.create_publisher(JointState,'/joint_states',10)
initial=json.loads((A/'initial.json').read_text())
state=JointState(name=c['joint_names'],position=[initial[n] for n in c['joint_names']])
def publish():
    state.header.stamp=publisher.get_clock().now().to_msg();pub.publish(state)
publisher.create_timer(.02,publish)
executor=SingleThreadedExecutor();executor.add_node(publisher)
worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
try:
    client=robot.create_client(GetPositionIK,'/compute_ik')
    if not client.wait_for_service(timeout_sec=40):raise RuntimeError('MoveIt startup failed')
    robot.state();time.sleep(1);robot.scene(apply=True)
    command=['python3','-m','doodle_ur7e.cli','--config',str(cfg),'baymax','--image',str(ROOT/'images/external/smiley.png'),
             '--images-dir',str(OUT/'images'),'--paper',str(ROOT/'runs/isaac/paper.yaml'),'--hover','0']
    good=subprocess.run(command,capture_output=True,text=True,timeout=90)
    (OUT/'planning.log').write_text(good.stdout+good.stderr)
    if good.returncode:raise RuntimeError('image CLI planning failed: '+good.stderr)
    plans=list((OUT/'images/processed').glob('*/robot_plan/plan.json'))
    plan=json.loads(plans[0].read_text())
    assert plan['fraction']==1. and plan['executed'] is False
    c['world_boxes'].append({'id':'doodle_image_test_blocker','center':[0.,.5,.34],'size':[.4,.3,.4]})
    cfg.write_text(yaml.safe_dump(c));robot.scene(apply=True)
    blocked=subprocess.run(command+['--execute','--supervised'],capture_output=True,text=True,timeout=90)
    (OUT/'blocked.log').write_text(blocked.stdout+blocked.stderr)
    if blocked.returncode==0 or 'incomplete path rejected' not in blocked.stderr:
        raise RuntimeError('blocked image execution was not rejected at Cartesian planning: '+blocked.stderr)
    report={'passed':True,'image_cli_plan_fraction':plan['fraction'],'trajectory_points':plan['points'],
            'blocked_execution_rejected':True,'hardware_or_action_server_started':False}
    (OUT/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
finally:
    executor.shutdown();worker.join(timeout=3);publisher.destroy_node();robot.destroy_node();rclpy.shutdown()
    for child in reversed(children):
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
    log.close()
