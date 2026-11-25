#!/usr/bin/env python3
"""CPU contract test of the real scaled controller + our hardware plugin, not a physics test."""
import os,signal,subprocess,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time,Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
import yaml
ROOT=Path(__file__).resolve().parents[1];A=ROOT/'runs/isaac/assets'
if os.environ.get('ROS_DOMAIN_ID')!='231':raise RuntimeError('isolated domain required')
rclpy.init();n=Node('controller_contract_fixture')
names=yaml.safe_load((ROOT/'config/isaac.yaml').read_text())['joint_names'];q=[0.]*6;counter=0
pub=n.create_publisher(JointState,'/doodle_sim/joint_feedback',10);clock=n.create_publisher(Clock,'/clock',10)
def receive(msg):
    global q
    q=[msg.position[msg.name.index(j)] for j in names]
n.create_subscription(JointState,'/doodle_sim/joint_commands',receive,1)
def tick():
    global counter
    counter+=1;stamp=Time(sec=counter//100,nanosec=(counter%100)*10000000)
    clock.publish(Clock(clock=stamp));m=JointState(name=names,position=q,velocity=[0.]*6);m.header.stamp=stamp;pub.publish(m)
n.create_timer(.01,tick);executor=SingleThreadedExecutor();executor.add_node(n)
worker=threading.Thread(target=executor.spin);worker.start()
log=(ROOT/'runs/controller_contract.log').open('w')
p=subprocess.Popen(['ros2','run','controller_manager','ros2_control_node','--ros-args','--params-file',str(A/'robot_parameters.yaml'),'--params-file',str(A/'controllers.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def wait(f,seconds=10):
    end=time.monotonic()+seconds
    while not f.done() and time.monotonic()<end:time.sleep(.01)
    if not f.done():raise TimeoutError('controller contract timeout')
    return f.result()
try:
    subprocess.run(['ros2','run','controller_manager','spawner','joint_state_broadcaster','scaled_joint_trajectory_controller','--controller-manager-timeout','30'],check=True,timeout=45)
    client=ActionClient(n,FollowJointTrajectory,'/scaled_joint_trajectory_controller/follow_joint_trajectory')
    if not client.wait_for_server(timeout_sec=5):raise RuntimeError('controller action missing')
    goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names
    goal.trajectory.points=[JointTrajectoryPoint(positions=[.002]*6,velocities=[0.]*6,time_from_start=Duration(sec=10))]
    handle=wait(client.send_goal_async(goal));assert handle.accepted
    result=handle.get_result_async();time.sleep(.2)
    canceled=wait(handle.cancel_goal_async());assert canceled.return_code==0
    terminal=wait(result);assert terminal.status==GoalStatus.STATUS_CANCELED
    print('PASS: official scaled controller + Isaac hardware plugin accepted and confirmed cancellation.')
finally:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    executor.shutdown();worker.join(timeout=3);n.destroy_node();rclpy.shutdown();log.close()
