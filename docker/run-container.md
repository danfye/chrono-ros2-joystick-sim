# Docker Runtime Notes

This note records the Docker runtime command used by the demo.

## Basic Launch

```bash
xhost +

docker run -it \
  --name chrono_sim_demo \
  --privileged \
  --net=host \
  -v /dev/input:/dev/input \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/dri:/dev/dri \
  -e DISPLAY="$DISPLAY" \
  chrono_gpu_ready bash
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

## Safer Follow-Up

For development, `--privileged` is convenient. For a tighter runtime, identify
the actual joystick event device and pass only that device:

```bash
ls -l /dev/input/by-id
```

Then mount the specific device with `--device` and the necessary group
permissions.
