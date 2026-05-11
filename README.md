# Chrono ROS 2 Joystick Simulation

A Docker-based Project Chrono and ROS 2 co-simulation demo for controlling a
Chrono HMMWV vehicle model with a physical joystick.

The project demonstrates a hardware-in-the-loop style control path:

```text
Joystick -> Linux /dev/input -> Docker container -> ROS 2 joy_node -> /joy
         -> C++ subscriber -> Chrono vehicle driver inputs -> HMMWV simulation
```

## Features

- Project Chrono vehicle dynamics simulation based on the `HMMWV_Full` model.
- ROS 2 Humble joystick input integration through `sensor_msgs/msg/Joy`.
- Docker runtime setup for HID device passthrough and X11 visualization.
- Primitive-based vehicle visualization to reduce missing-mesh issues inside
  containers.
- Flat terrain and grid visualization for a lightweight reproducible demo.

## System Overview

The runtime contains two main processes:

- `joy_node`, which reads the joystick through Linux input devices and publishes
  normalized joystick values on `/joy`.
- `chrono_joy_driver`, which subscribes to `/joy`, maps joystick axes to steering,
  throttle, and braking, and advances the Chrono HMMWV simulation.

Default input mapping:

| Control | Source | Range |
| --- | --- | --- |
| Steering | `axes[0]` | `[-1.0, 1.0]` |
| Throttle | `max(0, axes[1])` | `[0.0, 1.0]` |
| Braking | `max(0, -axes[1])` | `[0.0, 1.0]` |

## Environment

The original implementation was validated on Ubuntu with:

- ROS 2 Humble Hawksbill
- Project Chrono `release/9.0`
- Docker
- X11 display forwarding
- Linux joystick device exposed under `/dev/input`

The demo is most practical on a Linux host. GPU acceleration may need additional
driver-specific configuration; software OpenGL fallback is documented below.

## Install Dependencies

Install ROS 2 Humble on Ubuntu 22.04 following the official ROS 2 installation
instructions, then load the environment:

```bash
source /opt/ros/humble/setup.bash
```

Build Project Chrono from the stable 9.0 release with the Vehicle and Irrlicht
modules enabled:

```bash
sudo apt install build-essential cmake git \
  libirrlicht-dev libgl1-mesa-dev libglu1-mesa-dev

git clone https://github.com/projectchrono/chrono.git
cd chrono
git checkout release/9.0

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_MODULE_VEHICLE=ON \
  -DENABLE_MODULE_IRRLICHT=ON \
  -DENABLE_MODULE_POSTPROCESS=ON \
  ..

make -j"$(nproc)"
sudo make install
```

## Docker Runtime

The container needs access to:

- `/dev/input` for the joystick/HID devices.
- `/tmp/.X11-unix` and `DISPLAY` for X11 visualization.
- `/dev/dri` for direct rendering when available.
- host networking for simple ROS 2 discovery.

Example launch command:

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

For a tighter setup, replace `--privileged` with explicit device and group
permissions after confirming which `/dev/input/event*` device belongs to the
joystick.

## Run

In one terminal, start the ROS 2 joystick node:

```bash
source /opt/ros/humble/setup.bash
ros2 run joy joy_node
```

Check whether joystick messages are published:

```bash
ros2 topic echo /joy
```

Build and run the Chrono driver from `src/simplified_driver.cpp` inside the
environment where Chrono and ROS 2 are available. A typical ROS 2 package layout
can place this file under a package such as `chrono_sim_control/src/`.

## OpenGL Fallback

If the container reports errors such as:

```text
libGL error: failed to load driver: nouveau
```

try software rendering:

```bash
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_SOFTWARE=1
```

This is slower than GPU rendering, but is useful for validating the simulation
and ROS 2 control pipeline in a reproducible container.

## Troubleshooting

### `joy_node` starts but `/joy` has no output

Linux joystick support often exposes modern controllers through
`/dev/input/event*`, not only `/dev/input/js0`. Make sure the container can read
the host input devices:

```bash
ls /dev/input
ros2 topic echo /joy
```

If no events are visible, restart the container with `/dev/input` mounted and
the correct permissions.

### Vehicle mesh assets are missing inside Docker

The demo uses Chrono primitive visualization for chassis, tires, suspension,
steering, and wheels. This avoids relying on external mesh files and keeps the
container demo more robust.

## Repository Structure

```text
.
├── README.md
├── docs/
│   └── implementation-report.md
├── docker/
│   └── run-container.md
├── src/
│   └── simplified_driver.cpp
└── LICENSE
```

## License

This repository is prepared with the MIT License. Check the licenses of ROS 2,
Project Chrono, Irrlicht, and any assets or containers used in your deployment.
