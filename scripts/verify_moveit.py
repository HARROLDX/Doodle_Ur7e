#!/usr/bin/env python3
"""Offline actual UR7e kinematics/planner test. No driver or controller is launched."""
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rcl_interfaces.srv import GetParameters
from doodle_ur7e.core import load_config,load_strokes
from doodle_ur7e.ros import Robot,pose


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '231' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('requires isolated test domain 231 + localhost only')
    root=Path(__file__).resolve().parents[1]
    out=root/'runs/real_moveit'; out.mkdir(parents=True,exist_ok=True)
    with (out/'move_group.log').open('w') as log:
        process=subprocess.Popen(['ros2','launch','ur_moveit_config','ur_moveit.launch.py','ur_type:=ur7e',
                                  'launch_rviz:=false','launch_servo:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init(); c=load_config(root/'config/lab.yaml'); robot=Robot(c)
        publisher=Node('offline_joint_state'); pub=publisher.create_publisher(JointState,'/joint_states',10)
        rsp=None
        worker=None
        executor=SingleThreadedExecutor(); executor.add_node(publisher)
        try:
            cli=robot.create_client(GetPositionIK,'/compute_ik')
            if not cli.wait_for_service(timeout_sec=40.): raise RuntimeError('MoveIt did not start')
            params=robot.call(GetParameters,'/move_group/get_parameters',GetParameters.Request(names=['robot_description']))
            description=params.values[0].string_value
            param_file=out/'robot_description.yaml'
            param_file.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':description}}}))
            rsp=subprocess.Popen(['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(param_file)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            robot.until(lambda: robot.buffer.can_transform('world',c['base_frame'],rclpy.time.Time()),10.)
            req=GetPositionIK.Request()
            req.ik_request.group_name=c['group']; req.ik_request.ik_link_name=c['ik_link']
            req.ik_request.pose_stamped=PoseStamped()
            req.ik_request.pose_stamped.header.frame_id=c['base_frame']
            req.ik_request.pose_stamped.pose=pose([0.,.5,.065],c['link_quaternion'])
            req.ik_request.robot_state.joint_state=JointState(name=c['joint_names'],position=[4.72,-1.85,-1.425,-1.405,1.593,-3.141])
            req.ik_request.timeout.sec=10; req.ik_request.avoid_collisions=True
            state=req.ik_request.robot_state.joint_state
            def publish():
                state.header.stamp=publisher.get_clock().now().to_msg(); pub.publish(state)
            publisher.create_timer(.02,publish)
            worker=threading.Thread(target=executor.spin,daemon=True); worker.start()
            robot.state()
            time.sleep(3.0)  # allow MoveIt's separate TF listener to discover static frames
            res=robot.result(cli.call_async(req),20.)
            if res.error_code.val != 1: raise RuntimeError(f'initial IK failed {res.error_code.val}')
            state=res.solution.joint_state
            robot.scene(apply=True)
            cal=yaml.safe_load((root/'examples/paper.synthetic.yaml').read_text())
            lines=load_strokes(root/'examples/strokes.json')
            robot.plan(lines,cal,.10,out/'air',execute=False)
            robot.plan(lines,cal,0.,out/'contact',execute=False)
            c['world_boxes'].append({'id':'doodle_test_blocker','center':[0.,.5,-.01],'size':[.3,.3,.5]})
            robot.scene(apply=True)
            try:
                robot.plan(lines,cal,.10,out/'blocked',execute=False)
            except ValueError as error:
                if 'incomplete path' not in str(error): raise
                print('Deliberate obstruction correctly rejected:',error)
            else:
                raise AssertionError('MoveIt accepted deliberate obstruction')
            print('Actual UR7e MoveIt IK, collision scene, Cartesian planning, timing and final FK passed; no driver/action sent.')
        finally:
            robot.destroy_node()
            executor.shutdown()
            if worker: worker.join(timeout=3)
            publisher.destroy_node()
            rclpy.shutdown()
            if rsp:
                os.killpg(rsp.pid,signal.SIGINT); rsp.wait(timeout=10)
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=5)

if __name__=='__main__': main()
