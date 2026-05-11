# Project Chrono and ROS 2 Co-Simulation with Joystick Control

This document summarizes the implementation of a Docker-based Project Chrono and
ROS 2 co-simulation system that uses a physical joystick to control a Chrono
HMMWV vehicle model.

## 1. Goal

The goal is to build a reproducible simulation environment where a Linux host
passes joystick input and graphics display access into a Docker container. ROS 2
handles joystick messages, while a C++ Chrono program subscribes to those
messages and applies them to the simulated HMMWV vehicle.

The resulting flow supports a hardware-in-the-loop style test setup:

```text
Host joystick
  -> Linux input subsystem
  -> Docker device passthrough
  -> ROS 2 joy_node
  -> /joy topic
  -> Chrono driver subscriber
  -> HMMWV vehicle dynamics and visualization
```

## 2. Dependency Preparation

### 2.1 ROS 2

The implementation uses ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS. The typical
setup is:

1. Add the ROS 2 package repository and GPG key.
2. Install `ros-humble-desktop`.
3. Source ROS 2 in the shell environment:

```bash
source /opt/ros/humble/setup.bash
```

The joystick pipeline depends on the `joy` package and the
`sensor_msgs/msg/Joy` message type.

### 2.2 Project Chrono

The implementation targets Project Chrono `release/9.0` to keep the C++ API
stable. The required Chrono modules are:

- `VEHICLE`
- `IRRLICHT`
- `POSTPROCESS`

Build outline:

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

## 3. Docker Deployment

The Docker container provides a consistent runtime for ROS 2, Chrono libraries,
and visualization dependencies. The host must pass through two categories of
resources:

- Input devices: `/dev/input` for joystick events.
- Display resources: X11 socket, `DISPLAY`, and optionally `/dev/dri` for GPU
  rendering.

Example:

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

`--net=host` simplifies ROS 2 communication during local testing. The
`--privileged` flag is convenient for experiments, but a production setup should
replace it with narrower device permissions.

## 4. Implementation Architecture

### 4.1 Data Flow

The simulation uses ROS 2 as the control bus:

1. The physical joystick is detected by Linux under `/dev/input`.
2. Docker mounts `/dev/input` so the container can read the device.
3. `joy_node` publishes `sensor_msgs/msg/Joy` messages on `/joy`.
4. The Chrono C++ node subscribes to `/joy`.
5. Joystick axes are converted into Chrono `DriverInputs`.
6. The HMMWV model advances with the new steering, throttle, and braking inputs.

### 4.2 Vehicle Model

The C++ demo uses:

- `chrono::vehicle::hmmwv::HMMWV_Full`
- `ChContactMethod::NSC`
- `EngineModelType::SIMPLE`
- `TireModelType::TMEASY`
- A custom flat terrain derived from `ChTerrain`

The flat terrain keeps the demo focused on control and integration rather than
terrain modeling.

### 4.3 Joystick Mapping

The joystick subscriber maps axes to vehicle inputs:

```text
steering = axes[0]
throttle = max(0, axes[1])
braking  = max(0, -axes[1])
```

This mapping treats the second axis as a combined throttle/brake channel.
Positive values accelerate the vehicle and negative values brake it.

### 4.4 Visualization Strategy

Containerized visualization can fail when external mesh assets or GPU drivers are
not available. To make the demo more robust, the implementation uses primitive
geometry for visible vehicle components:

```cpp
my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);
my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
my_hmmwv.SetWheelVisualizationType(VisualizationType::PRIMITIVES);
```

A 200 x 200 ground grid is added to make motion easier to perceive.

## 5. Key Issues and Fixes

### 5.1 No `/joy` Output

Symptom: `joy_node` starts, but `ros2 topic echo /joy` shows no messages.

Cause: the container cannot access the Linux joystick event devices. Modern Linux
input often uses `/dev/input/event*`; relying only on `/dev/input/js0` may miss
the controller.

Fix:

```bash
docker run ... -v /dev/input:/dev/input ...
```

If necessary, use `--privileged` for initial testing, then tighten permissions
once the correct device node is known.

### 5.2 OpenGL Driver Error

Symptom: Irrlicht visualization fails with an OpenGL driver error such as:

```text
libGL error: failed to load driver: nouveau
```

Cause: the container does not have a compatible GPU driver stack.

Fallback:

```bash
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_SOFTWARE=1
```

This uses CPU rendering. It reduces performance but improves portability for
testing the control loop.

## 6. Result

The final system validates the complete path from a physical joystick to a
Chrono HMMWV simulation. The demo shows that ROS 2 can be used as the message
interface between HID input and a Project Chrono vehicle dynamics model inside a
Docker environment.

The most important implementation decisions are:

- Use Docker device passthrough for joystick access.
- Use X11 forwarding for visualization.
- Keep ROS 2 as the runtime control bus.
- Use primitive visualization to reduce asset-related failures.
- Provide software OpenGL fallback for machines without working container GPU
  acceleration.
