# Docker Runtime Notes

This note records the Docker workflow for the Chrono ROS 2 joystick simulation.

## Build the Image

From the repository root:

```bash
docker build -t chrono_ros2_joystick:humble -f docker/Dockerfile .
```

The image is based on Ubuntu 22.04 / ROS 2 Humble and builds Project Chrono
`release/9.0` with Vehicle, Irrlicht, and Postprocess modules enabled.

## Start the Container

```bash
chmod +x docker/run.sh
./docker/run.sh
```

The script expands to the same runtime strategy as:

```bash
xhost +local:root

docker run -it --rm \
  --name chrono_sim_demo \
  --privileged \
  --net=host \
  -v /dev/input:/dev/input \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/dri:/dev/dri \
  -v "$PWD:/workspace/chrono_sim_control" \
  -e DISPLAY="$DISPLAY" \
  -w /workspace \
  chrono_ros2_joystick:humble \
  bash
```

## Why These Flags Are Used

| Flag | Purpose |
| --- | --- |
| `--privileged` | Allows quick access to input devices during experiments. |
| `--net=host` | Simplifies ROS 2 discovery and topic communication. |
| `-v /dev/input:/dev/input` | Passes joystick/HID input devices into the container. |
| `-v /tmp/.X11-unix:/tmp/.X11-unix` | Passes the X11 Unix socket into the container. |
| `-v /dev/dri:/dev/dri` | Exposes direct rendering devices when available. |
| `-e DISPLAY="$DISPLAY"` | Points container GUI applications to the host display. |

## Build and Launch Inside the Container

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select chrono_sim_control
source install/setup.bash
ros2 launch chrono_sim_control joystick_hmmwv.launch.py
```

## OpenGL Software Fallback

If GPU-backed rendering fails:

```bash
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch chrono_sim_control joystick_hmmwv.launch.py
```

## Safer Follow-Up

For development, `--privileged` is convenient. For a tighter runtime, identify
the actual joystick event device and pass only that device:

```bash
ls -l /dev/input/by-id
```

Then mount the specific device with `--device` and the necessary group
permissions.
