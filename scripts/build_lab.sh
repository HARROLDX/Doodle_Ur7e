#!/usr/bin/env bash
set -euo pipefail
if [[ ${ROS_DISTRO:-} != humble ]]; then
  echo 'First: distrobox enter ros2 (course workstation, ROS 2 Humble)' >&2
  exit 1
fi
cd "$(dirname "$0")/.."
python3 -c 'import cv2, scipy, yaml, PIL; assert hasattr(cv2,"aruco")'
for pkg in moveit_msgs control_msgs cv_bridge tf2_ros ur7e_utils ur_moveit_config realsense2_camera; do
  ros2 pkg prefix "$pkg" >/dev/null
done
colcon build --symlink-install --packages-select doodle_ur7e
echo 'Build complete. Run: source install/setup.bash'
