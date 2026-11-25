#!/usr/bin/env bash
# Run on the A6000 host. GPU selection is explicit; obtain resource allocation first.
set -euo pipefail
cd "$(dirname "$0")/.."
: "${DOODLE_GPU:?Set DOODLE_GPU to the explicitly allocated GPU index}"
ROS_IMAGE=${DOODLE_ROS_IMAGE:-doodle-ur7e-sim:validated}
ISAAC_IMAGE=${DOODLE_ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}
RUN_ID="doodle-$(date +%Y%m%d-%H%M%S)"
if [[ -n "$(docker ps -q --filter "label=doodle.project=$PWD")" ]]; then
  echo 'A simulation for this checkout is already running; inspect its recorded container names.' >&2
  exit 1
fi
mkdir -p runs/isaac runs/isaac_runtime/{cache,logs,data}
if [[ -f runs/isaac/world_result.json || -f runs/isaac/ros_result.json ]]; then
  echo 'Archive runs/isaac before starting a new run; existing evidence will not be overwritten.' >&2
  exit 1
fi
common=(--label "doodle.project=$PWD" --network host -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=231 -e ROS_LOCALHOST_ONLY=1 -v "$PWD:/workspace" -w /workspace)
docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID=231 -e ROS_LOCALHOST_ONLY=1 -v "$PWD:/workspace" -w /workspace --entrypoint bash "$ROS_IMAGE" -lc 'source /opt/ros/humble/setup.bash; colcon build --symlink-install && source install/setup.bash && python3 sim/prepare.py && python3 sim/initial_pose.py && python3 -m doodle_ur7e.cli prepare runs/isaac/assets/smiley.png --output runs/isaac/strokes'
world="${RUN_ID}-world"; ros="${RUN_ID}-ros"
printf '%s\n' "$world" "$ros" > runs/isaac/containers.txt
# Use the same UID for ROS and Isaac DDS; group 1234 grants traversal of the upstream image.
# Writable cache/data/log mounts avoid changing the packaged simulator installation.
docker run -d --name "$world" --ipc shareable "${common[@]}" --user "$(id -u):$(id -g)" --group-add 1234 -e HOME=/tmp/doodle-home -v "$PWD/runs/isaac_runtime/cache:/isaac-sim/kit/cache" -v "$PWD/runs/isaac_runtime/logs:/isaac-sim/kit/logs" -v "$PWD/runs/isaac_runtime/data:/isaac-sim/kit/data" --gpus "device=$DOODLE_GPU" -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e ROS_DISTRO=humble -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp --entrypoint bash "$ISAAC_IMAGE" -lc 'mkdir -p "$HOME"; export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH; /isaac-sim/python.sh /workspace/sim/world.py > /workspace/runs/isaac/world.log 2>&1'
docker run -d --name "$ros" --ipc "container:$world" "${common[@]}" --user "$(id -u):$(id -g)" -e HOME=/tmp --entrypoint bash "$ROS_IMAGE" -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash; python3 -u sim/run_ros.py > runs/isaac/ros.log 2>&1'
ros_exit=$(docker wait "$ros")
if [[ "$ros_exit" != 0 ]]; then
  # Stop only this invocation's simulator if ROS could not deliver its finish message.
  docker stop -t 10 "$world" >/dev/null
fi
world_exit=$(docker wait "$world")
if [[ "$ros_exit" != 0 || "$world_exit" != 0 ]]; then
  echo "Simulation failed: ROS=$ros_exit Isaac=$world_exit; inspect runs/isaac logs." >&2
  exit 1
fi
docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:/workspace" -w /workspace --entrypoint bash "$ROS_IMAGE" -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash; python3 sim/analyze.py'
# 2 samples per simulation second played at 24 FPS: clearly labeled 12x time lapse.
ffmpeg -y -framerate 24 -i runs/isaac/frames/%06d.jpg -vf "drawtext=text='Isaac Sim UR7e - simulation time lapse 12x':x=20:y=20:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.6" -c:v libx264 -crf 20 -pix_fmt yuv420p runs/isaac/drawing.mp4
docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:/workspace" -w /workspace --entrypoint python3 "$ROS_IMAGE" sim/video.py --speed 60
printf 'Finished: %s/runs/isaac/drawing_60x.mp4\n' "$PWD"
