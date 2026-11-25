#!/usr/bin/env python3
"""Run the real Humble planning and controller stack against Isaac topic hardware."""
import json, os, signal, subprocess, time
from pathlib import Path
import numpy as np
import yaml
from scene_geometry import MARKER_SURFACE
import rclpy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from doodle_ur7e.core import load_config,load_strokes
from doodle_ur7e.ros import Robot
from doodle_ur7e.shutdown import stop_signals
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'runs/isaac';A=OUT/'assets'
if os.environ.get('ROS_DOMAIN_ID')!='231':raise RuntimeError('isolated domain required')
children=[];logs=[]
def start(name,cmd):
    log=(OUT/(name+'.log')).open('w');logs.append(log)
    child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(child);return child

def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);robot=Robot(load_config(ROOT/'config/isaac.yaml'))
    (OUT/'config.used.yaml').write_text(yaml.safe_dump(robot.c,sort_keys=False))
    record=robot.create_publisher(Bool,'/doodle_sim/record',10)
    finish=robot.create_publisher(Bool,'/doodle_sim/finish',10)
    success=False
    try:
        start('state_publisher',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(A/'robot_parameters.yaml')])
        start('move_group',['ros2','run','moveit_ros_move_group','move_group','--ros-args','--params-file',str(A/'moveit_parameters.yaml')])
        # Do not activate hardware until actual simulator feedback exists.
        feedback=[]
        sub=robot.create_subscription(JointState,'/doodle_sim/joint_feedback',lambda m:feedback.append(m),qos_profile_sensor_data)
        robot.until(lambda:bool(feedback),600.)
        robot.destroy_subscription(sub)
        start('controller_manager',['ros2','run','controller_manager','ros2_control_node','--ros-args','--params-file',str(A/'robot_parameters.yaml'),'--params-file',str(A/'controllers.yaml')])
        subprocess.run(['ros2','run','controller_manager','spawner','joint_state_broadcaster','scaled_joint_trajectory_controller','--controller-manager-timeout','60'],check=True,timeout=90)
        robot.state();robot.controller_check();robot.scene(apply=True)
        print('STACK_READY',flush=True)
        robot.until(lambda:robot.image is not None and robot.info is not None,60.)
        # Let sensor rendering settle before the strict 12-sample calibration.
        ready=time.monotonic()+3.
        while time.monotonic()<ready:rclpy.spin_once(robot,timeout_sec=.05)
        import cv2
        pixels,_,_=robot.capture();cv2.imwrite(str(OUT/'camera_before.png'),pixels)
        robot.calibrate(OUT/'paper.yaml')
        cal=yaml.safe_load((OUT/'paper.yaml').read_text())
        # Ground truth is used ONLY as an independent calibration acceptance check.
        error=float(np.linalg.norm(np.array(cal['position'])-[-.075,.425,MARKER_SURFACE]))
        if error>.002: raise ValueError(f'rendered-camera calibration error {error:.6f} m >2mm')
        cal['verified']=True;cal['simulation_position_error_m']=error
        (OUT/'paper.yaml').write_text(yaml.safe_dump(cal))
        print('CAMERA_CALIBRATION_PASSED',error,flush=True)
        lines=load_strokes(OUT/'strokes/strokes.json')
        record.publish(Bool(data=True))
        robot.plan(lines,cal,0.,OUT/'drawing',execute=True)
        record.publish(Bool(data=False));success=True
        print('ISAAC_DRAWING_SUCCEEDED',flush=True)
    finally:
        # Only the task-owned simulated world responds to this topic.
        for _ in range(5):
            finish.publish(Bool(data=True));rclpy.spin_once(robot,timeout_sec=.1)
        robot.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try: child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        for log in logs:log.close()
        (OUT/'ros_result.json').write_text(json.dumps({'success':success},indent=2))
if __name__=='__main__':
    with stop_signals():main()
