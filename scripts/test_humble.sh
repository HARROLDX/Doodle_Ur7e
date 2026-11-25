#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=231
colcon build --symlink-install --packages-select doodle_ur7e
source install/setup.bash
python3 -m pytest tests -q
ros2 run doodle_ur7e doodle --config config/lab.yaml preview examples/strokes.json --paper examples/paper.synthetic.yaml --output runs/smoke
ros2 run doodle_ur7e doodle --config config/lab.yaml board --output runs/board.svg
