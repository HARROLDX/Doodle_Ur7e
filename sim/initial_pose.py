"""Calculate the initial observation posture using the same MoveIt IK model (no actuation)."""
import json, subprocess, time, signal, os
from pathlib import Path
import rclpy
from scene_geometry import OBSERVATION_WRIST
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from doodle_ur7e.core import load_config
from doodle_ur7e.ros import pose
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'runs/isaac/assets'
log=(out/'initial_ik.log').open('w')
p=subprocess.Popen(['ros2','run','moveit_ros_move_group','move_group','--ros-args','--params-file',str(out/'moveit_parameters.yaml'),'-p','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rsp=subprocess.Popen(['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(out/'robot_parameters.yaml'),'-p','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=Node('initial_simulation_pose');c=load_config(ROOT/'config/isaac.yaml')
try:
    client=n.create_client(GetPositionIK,'/compute_ik')
    if not client.wait_for_service(timeout_sec=45): raise RuntimeError('IK startup failed')
    time.sleep(1.)
    req=GetPositionIK.Request();req.ik_request.group_name=c['group'];req.ik_request.ik_link_name=c['ik_link']
    req.ik_request.pose_stamped.header.frame_id=c['base_frame'];req.ik_request.pose_stamped.pose=pose(OBSERVATION_WRIST,c['link_quaternion'])
    req.ik_request.robot_state.joint_state=JointState(name=c['joint_names'],position=[4.72,-1.85,-1.425,-1.405,1.593,-3.141])
    req.ik_request.timeout.sec=10
    f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=20)
    result=f.result()
    if result is None or result.error_code.val!=1: raise RuntimeError('initial IK failed')
    (out/'initial.json').write_text(json.dumps(dict(zip(result.solution.joint_state.name,result.solution.joint_state.position)),indent=2))
finally:
    n.destroy_node();rclpy.shutdown();os.killpg(rsp.pid,signal.SIGINT);rsp.wait(timeout=10);os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
