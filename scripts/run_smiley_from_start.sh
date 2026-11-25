#!/usr/bin/env bash
# Run inside the ROS2 Distrobox. Every motion is freshly planned.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
trial_config="${CONFIG_FILE:-config/local.yaml}"
trial_paper="${PAPER_FILE:-runs/paper.yaml}"
trial_start="${START_FILE:-runs/drawing_start.json}"
trial_strokes="${STROKES_FILE:-runs/smiley/strokes.json}"
for trial_file in "$trial_config" "$trial_paper" "$trial_start" "$trial_strokes"; do
  if [[ ! -f "$trial_file" ]]; then
    printf 'Missing local input: %s\nComplete START.md setup before motion.\n' "$trial_file" >&2
    exit 1
  fi
done
trial_speed_response="$(timeout 15s ros2 service call /io_and_status_controller/set_speed_slider \
  ur_msgs/srv/SetSpeedSliderFraction '{speed_slider_fraction: 0.8}')"
printf '%s\n' "$trial_speed_response"
if [[ "$trial_speed_response" != *'success=True'* ]]; then
  printf 'Speed slider change not confirmed; no trajectory sent.\n' >&2
  exit 1
fi
# Only a successful, terminal return permits the draw command below.
python3 scripts/return_drawing_start.py --config "$trial_config" \
  --start "$trial_start" --paper "$trial_paper" --execute --supervised
ros2 run doodle_ur7e doodle --config "$trial_config" draw \
  "$trial_strokes" --paper "$trial_paper" \
  --hover 0 --execute --supervised \
  --output "runs/smiley-contact-$(date +%Y%m%d-%H%M%S)"
