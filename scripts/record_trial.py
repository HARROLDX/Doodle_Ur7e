"""Passive RGB video, timestamps and ROS bag recorder. Never commands motion."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import signal
import subprocess
import time
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from doodle_ur7e.core import load_config

TOPICS = ['/joint_states', '/tf', '/tf_static', '/camera/camera/color/camera_info',
          '/scaled_joint_trajectory_controller/controller_state',
          '/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/status',
          '/speed_scaling_state_broadcaster/speed_scaling',
          '/io_and_status_controller/robot_mode', '/io_and_status_controller/safety_mode',
          '/io_and_status_controller/robot_program_running',
          '/force_torque_sensor_broadcaster/wrench', '/tcp_pose_broadcaster/pose',
          '/display_planned_path', '/rosout']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--duration', type=float, default=1200.)
    p.add_argument('--config', default='config/local.yaml')
    p.add_argument('--paper', default='runs/paper.yaml')
    p.add_argument('--start', default='runs/drawing_start.json')
    p.add_argument('--strokes', default='runs/smiley/strokes.json')
    a = p.parse_args()
    if not 0 < a.duration <= 7200:
        p.error('duration must be 0..7200 seconds')
    c = load_config(a.config)
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    for source, name in [(a.config, 'config.yaml'), (a.paper, 'paper.yaml'),
                         (a.start, 'drawing_start.json'), (a.strokes, 'strokes.json')]:
        if Path(source).is_file():
            shutil.copy2(source, out/name)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('doodle_trial_recorder')
    bridge = CvBridge()
    writer = None
    bag = None
    frames = 0
    last = 0.
    stopping = False
    started = time.monotonic()
    stamps = (out/'video_timestamps.csv').open('w', newline='')
    rows = csv.writer(stamps)
    rows.writerow(['frame', 'image_ros_sec', 'image_ros_nanosec', 'received_unix_sec'])
    log = (out/'rosbag.log').open('w')

    def stop(*_):
        nonlocal stopping
        stopping = True

    def image(msg):
        nonlocal writer, frames, last
        now = time.monotonic()
        if now-last < .1:
            return
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if writer is None:
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(str(out/'realsense.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 10., (w, h))
            if not writer.isOpened():
                raise RuntimeError('video encoder unavailable')
        writer.write(frame)
        rows.writerow([frames, msg.header.stamp.sec, msg.header.stamp.nanosec, time.time()])
        stamps.flush()
        frames += 1
        last = now
        if frames == 1:
            print('VIDEO_RECORDING_READY', flush=True)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node.create_subscription(Image, c['camera']['image_topic'], image, qos_profile_sensor_data)
    try:
        topics = [c['camera']['info_topic'] if t == '/camera/camera/color/camera_info' else t for t in TOPICS]
        bag = subprocess.Popen(['ros2', 'bag', 'record', '--include-hidden-topics',
                                '-o', str(out/'robot_data'), *topics], stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
        print(f'Recording {out}, limit {a.duration:g}s; Ctrl+C stops recording only.', flush=True)
        while rclpy.ok() and not stopping and time.monotonic()-started < a.duration:
            if bag.poll() is not None:
                raise RuntimeError(f'rosbag exited ({bag.returncode}); see {out}/rosbag.log')
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if bag is not None and bag.poll() is None:
            # This process group contains only our recorder, never robot nodes.
            import os
            os.killpg(bag.pid, signal.SIGINT)
            try:
                bag.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGTERM)
                bag.wait(timeout=5)
        if writer is not None:
            writer.release()
        stamps.close()
        log.close()
        (out/'summary.json').write_text(json.dumps(dict(frames=frames,
            elapsed_s=time.monotonic()-started, video_note='10fps nominal; use CSV for actual timestamps'), indent=2))
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
