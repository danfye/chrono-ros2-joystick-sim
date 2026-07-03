#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-chrono_ros2_joystick:humble}"
CONTAINER_NAME="${CONTAINER_NAME:-chrono_sim_demo}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null
fi

docker run -it --rm \
  --name "${CONTAINER_NAME}" \
  --privileged \
  --net=host \
  -v /dev/input:/dev/input \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/dri:/dev/dri \
  -v "${PROJECT_DIR}:/workspace/chrono_sim_control" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -w /workspace \
  "${IMAGE_NAME}" \
  bash
