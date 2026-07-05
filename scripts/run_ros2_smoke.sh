#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

image_name="${IMAGE_NAME:-chrono_ros2_deps:humble}"
verify_root="${VERIFY_ROOT:-/tmp/chrono_ros2_verify}"
chrono_install="${CHRONO_INSTALL:-${verify_root}/chrono_install}"
headless_runtime_seconds="${HEADLESS_RUNTIME_SECONDS:-0.05}"

if [[ ! -d "${chrono_install}/lib/cmake/Chrono" ]]; then
  echo "Chrono install was not found at ${chrono_install}" >&2
  echo "Set CHRONO_INSTALL or build Chrono into ${chrono_install} first." >&2
  exit 1
fi

mkdir -p "${verify_root}/ros_ws/src"

docker run --rm \
  -v "${repo_root}:${verify_root}/ros_ws/src/chrono_sim_control:ro" \
  -v "${verify_root}:${verify_root}" \
  "${image_name}" \
  bash -lc "
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    export LD_LIBRARY_PATH='${chrono_install}/lib':\"\${LD_LIBRARY_PATH:-}\"
    cd '${verify_root}/ros_ws'
    colcon build --packages-select chrono_sim_control --symlink-install \
      --cmake-args \
      -DChrono_DIR='${chrono_install}/lib/cmake/Chrono' \
      -DCMAKE_PREFIX_PATH='${chrono_install}'
    colcon test --packages-select chrono_sim_control
    colcon test-result --verbose
    source install/setup.bash
    ros2 launch chrono_sim_control joystick_hmmwv.launch.py \
      start_joy_node:=false \
      headless:=true \
      max_runtime_seconds:=${headless_runtime_seconds} \
      chrono_data_path:='${chrono_install}/share/chrono/data/' \
      config_file:='${verify_root}/ros_ws/src/chrono_sim_control/config/chrono_joy_driver.yaml'
  "
