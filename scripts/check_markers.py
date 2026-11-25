"""Read-only multi-frame check using the project's ArUco detector settings."""
import argparse
from collections import Counter
import time
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from doodle_ur7e.core import load_config
from doodle_ur7e.vision import dictionary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/local.yaml')
    parser.add_argument('--frames', type=int, default=90)
    parser.add_argument('--timeout', type=float, default=15.)
    args = parser.parse_args()
    if args.frames <= 0 or args.timeout <= 0:
        parser.error('frames and timeout must be positive')
    config = load_config(args.config)
    rclpy.init(args=[])
    node = Node('doodle_multiframe_marker_check')
    bridge = CvBridge()
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    counts = Counter()
    combinations = Counter()
    frames = 0
    stamps = set()

    def callback(msg):
        nonlocal frames
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if frames >= args.frames or stamp in stamps:
            return
        age = (node.get_clock().now().nanoseconds-(stamp[0]*10**9+stamp[1]))/1e9
        if not 0 <= age < .5:
            return
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                           dictionary(), parameters=params)
        seen = tuple(sorted(int(i) for i in ids.ravel())) if ids is not None else ()
        counts.update(set(seen))
        combinations[seen] += 1
        stamps.add(stamp)
        frames += 1

    node.create_subscription(Image, config['camera']['image_topic'], callback, qos_profile_sensor_data)
    deadline = time.monotonic()+args.timeout
    try:
        while rclpy.ok() and frames < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        print(f'Fresh unique frames: {frames}/{args.frames}')
        for i in range(4):
            print(f'ID {i}: {counts[i]}/{frames}')
        complete = combinations[(0, 1, 2, 3)]
        print(f'All four together: {complete}/{frames}')
        if frames != args.frames or complete != frames:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
