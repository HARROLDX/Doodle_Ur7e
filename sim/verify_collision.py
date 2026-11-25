#!/usr/bin/env python3
"""Check every planned joint sample against the configured world and attachments."""
import json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from doodle_ur7e.core import load_config
from doodle_ur7e.ros import Robot
ROOT=Path(__file__).resolve().parents[1];A=ROOT/'runs/isaac/assets'
if os.environ.get('ROS_DOMAIN_ID')!='231':raise RuntimeError('isolated domain required')
c=load_config(ROOT/'config/isaac.yaml');c['use_sim_time']=False
plan=json.loads((ROOT/'runs/isaac/drawing/plan.json').read_text())
log=(ROOT/'runs/collision_revalidation.log').open('w');children=[]
for package,executable,params in [('robot_state_publisher','robot_state_publisher','robot_parameters.yaml'),('moveit_ros_move_group','move_group','moveit_parameters.yaml')]:
    children.append(subprocess.Popen(['ros2','run',package,executable,'--ros-args','--params-file',str(A/params),'-p','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
rclpy.init();robot=Robot(c)
try:
    client=robot.create_client(GetStateValidity,'/check_state_validity')
    if not client.wait_for_service(timeout_sec=40):raise RuntimeError('MoveIt failed to start')
    time.sleep(1);robot.scene(apply=True)
    for i,positions in enumerate(plan['joint_positions']):
        req=GetStateValidity.Request();req.group_name=c['group'];req.robot_state.is_diff=True
        req.robot_state.joint_state=JointState(name=plan['joint_names'],position=positions)
        response=robot.result(client.call_async(req))
        if not response.valid:raise RuntimeError(f'collision at trajectory sample {i}: {response.contacts}')
    (ROOT/'runs/isaac/collision_validation.json').write_text(json.dumps({
        'passed':True,'samples':len(plan['joint_positions']),
        'world_boxes':c['world_boxes'],'attached_boxes':c['attached_boxes']},indent=2))
    print(f'PASS: all {len(plan["joint_positions"])} trajectory samples collision-free with configured world and attachment enclosures.')
finally:
    robot.destroy_node();rclpy.shutdown()
    for child in reversed(children):
        os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
    log.close()
