#!/usr/bin/env python3
"""Read and validate an actual rendered RGB-D stream; no robot commands."""
import json,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
rclpy.init();n=Node('doodle_depth_check');received=[]
n.create_subscription(Image,'/camera/camera/depth/image_rect_raw',received.append,qos_profile_sensor_data)
try:
    end=time.monotonic()+30
    while not received and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    if not received:raise RuntimeError('no depth frame')
    m=received[-1];assert m.encoding=='32FC1'
    z=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width)
    valid=np.isfinite(z)&(z>0)&(z<10)
    assert valid.mean()>.8
    print(json.dumps({'width':m.width,'height':m.height,'encoding':m.encoding,'frame':m.header.frame_id,'valid_fraction':float(valid.mean()),'median_depth_m':float(np.median(z[valid]))},indent=2))
finally:n.destroy_node();rclpy.shutdown()
