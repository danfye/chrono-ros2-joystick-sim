# Project Chrono and ROS 2 Co-Simulation with Joystick Control

This document summarizes the implementation of a Docker-based Project Chrono and
ROS 2 co-simulation system that uses a physical joystick to control a Chrono
HMMWV vehicle model.

## 1. Goal

The goal is to build a reproducible simulation environment where a Linux host
passes joystick input and graphics display access into a Docker container. ROS 2
handles joystick messages, while a C++ Chrono program subscribes to those
messages and applies them to the simulated HMMWV vehicle.

The current prototype is intended to be more than a visual demo. The joystick
mapping, safety policy, and validation steps are configured through YAML so that
different controllers and repeated experiments can be documented without
changing C++ source code.

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
5. Joystick axes and optional buttons are converted into Chrono `DriverInputs`.
6. Timeout and deadman checks prevent stale or disabled input from continuing to
   drive the vehicle.
7. The HMMWV model advances with the new steering, throttle, and braking inputs.

### 4.2 Vehicle Model

The C++ demo uses:

- `chrono::vehicle::hmmwv::HMMWV_Full`
- `ChContactMethod::NSC`
- `EngineModelType::SIMPLE`
- `TireModelType::TMEASY`
- A custom flat terrain derived from `ChTerrain`

The flat terrain keeps the demo focused on control and integration rather than
terrain modeling.

### 4.3 Joystick Mapping and Parameters

The original demo mapping is preserved as the default YAML configuration:

```text
steering = axes[0]
throttle = max(0, axes[1])
braking  = max(0, -axes[1])
```

This mapping treats the second axis as a combined throttle/brake channel.
Positive values accelerate the vehicle and negative values brake it.

The configuration file is `config/chrono_joy_driver.yaml`, and the launch file
passes it into `chrono_joy_driver`:

```bash
ros2 launch chrono_sim_control joystick_hmmwv.launch.py \
  config_file:=/workspace/chrono_sim_control/config/chrono_joy_driver.yaml
```

Additional experiment profiles can be selected with the same launch argument.
The repository includes:

- `config/profiles/combined_axis_coast.yaml`: original combined-axis behavior
  with timeout/deadman clearing input to zero.
- `config/profiles/split_triggers_deadman_brake.yaml`: split trigger mapping,
  example enable/deadman buttons, and `safe_brake_value: 0.4`.

The parameter set is designed to cover common controllers:

| Group | Parameters | Reason |
| --- | --- | --- |
| Topic and timing | `joy_topic`, `step_size` | Keeps ROS topic naming and Chrono integration step explicit. |
| Steering | `steering_axis`, `invert_steering`, `steering_scale` | Supports left/right sign differences and limiting steering authority. |
| Combined throttle/brake | `throttle_brake_axis`, `invert_throttle`, `invert_brake` | Supports the original single signed axis behavior. |
| Independent throttle/brake | `throttle_axis`, `brake_axis`, `invert_throttle`, `invert_brake` | Supports gamepads whose triggers appear as separate axes. |
| Scaling and filtering | `throttle_scale`, `brake_scale`, `deadzone` | Handles noisy neutral positions and different axis ranges. |
| Safety | `joystick_timeout_seconds`, `safe_brake_on_timeout`, `enable_button`, `deadman_button`, `safe_brake_value` | Prevents stale joystick commands and allows operator-held enable/deadman buttons. |
| Chrono data | `chrono_data_path` | Allows native or custom Chrono installations without recompiling. |

The recommended calibration method is to run `ros2 topic echo /joy`, move one
physical control at a time, record the axis or button index, and then update the
YAML file. This avoids recompilation and makes the exact experiment mapping
auditable.

### 4.4 Timeout and Deadman Safety

Joystick control is unsafe if the vehicle continues to use the last nonzero
command after the controller disconnects or the operator releases control. The
driver therefore uses a safety output whenever input is not currently valid:

```text
steering = 0
throttle = 0
braking  = safe_brake_value
```

The safety output is expected in these cases:

- No `/joy` message has arrived within `joystick_timeout_seconds`.
- `enable_button` or `deadman_button` is configured and one of them is not held.
- A configured axis or button index is outside the current message bounds.

Missing configured axes or buttons are treated as invalid message layouts. They
force the safety output and do not refresh the timeout timer, which makes a bad
YAML profile fail safe instead of silently extending stale control.

Two operating styles are supported:

- `safe_brake_value: 0.0` for zero-input coast during early bench validation.
- A positive `safe_brake_value`, for example `0.4`, when timeout or deadman
  release should actively slow the simulated HMMWV.

This policy makes the prototype easier to reproduce because every run has an
explicit definition of stale-input behavior.

### 4.5 Unit-Testable Mapper

The joystick conversion logic lives in `JoystickMapper`, separate from ROS 2 and
Chrono. This keeps the safety behavior testable without a simulator window or a
physical controller. The test suite covers:

- deadzone and axis clamping
- invalid axis and invalid button indices
- combined positive throttle and negative braking
- split throttle/brake axes
- inversion and scaling
- timeout before first message and after stale input
- enable/deadman button requirements

### 4.6 Visualization Strategy

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

### 5.3 Controller Axis Direction Does Not Match the Vehicle

Symptom: pushing the stick right steers left, or pressing a trigger applies the
opposite command.

Fix: inspect `/joy` with:

```bash
ros2 topic echo /joy
```

Then adjust the relevant YAML field, for example `invert_steering: true` or
`invert_throttle: true`.

### 5.4 Vehicle Keeps Moving After Joystick Input Stops

Symptom: a stale throttle command remains after the publisher stops or a
controller disconnects.

Fix: set a finite timeout and choose the desired safety behavior:

```yaml
joystick_timeout_seconds: 0.5
safe_brake_value: 0.0
```

For a stronger stop behavior, set `safe_brake_value` to a positive value and
verify that the simulated vehicle slows when `/joy` messages stop.

## 6. Validation Plan

The recommended validation sequence keeps the Docker path unchanged and can be
run with or without physical hardware.

### 6.1 Build Validation

Inside the container:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select chrono_sim_control
source install/setup.bash
```

This confirms the ROS 2 Humble, Chrono 9.0, `ament_cmake`, launch, and install
paths still work.

Run the mapper unit tests:

```bash
colcon test --packages-select chrono_sim_control
colcon test-result --verbose
```

The mapper tests provide a fast regression check before hardware trials.

### 6.2 Synthetic `/joy` Validation

Run only the Chrono driver:

```bash
ros2 run chrono_sim_control chrono_joy_driver
```

Publish representative messages from another shell:

```bash
ros2 topic pub /joy sensor_msgs/msg/Joy \
  "{axes: [0.5, 0.8], buttons: []}" -r 10
```

Expected result: right steering and throttle.

```bash
ros2 topic pub /joy sensor_msgs/msg/Joy \
  "{axes: [0.0, -0.8], buttons: []}" -r 10
```

Expected result: braking with zero throttle.

For timeout validation, publish a single command and then stop:

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy \
  "{axes: [0.3, 0.7], buttons: []}"
```

Expected result: after `joystick_timeout_seconds`, steering and throttle return
to zero and braking becomes `safe_brake_value`.

For deadman validation, enable `enable_button` and `deadman_button`, then
compare messages where both buttons are pressed versus one released. Only the
held-button case should apply mapped driving input.

### 6.3 Physical Joystick Validation

Use the same Docker workflow as the demo:

```bash
chmod +x docker/run.sh
./docker/run.sh
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select chrono_sim_control
source install/setup.bash
ros2 launch chrono_sim_control joystick_hmmwv.launch.py
```

Before driving, confirm controller visibility:

```bash
ls /dev/input
ros2 topic echo /joy
```

Record the exact YAML file, Docker image tag, launch command, controller model,
and observed axis/button indices with each run.

## 7. Recommended Experiment Flow

1. Build the Docker image from `docker/Dockerfile`.
2. Start the container with `docker/run.sh`; keep the `/dev/input`, X11, and
   optional `/dev/dri` mounts unchanged.
3. Confirm `/joy` output before launching Chrono.
4. Calibrate the controller and commit or archive the YAML used for the run.
5. Validate deadzone, steering sign, throttle, braking, timeout, and optional
   deadman behavior with synthetic messages.
6. Run the Chrono HMMWV trial at low speed, using `safe_brake_value: 0.0`
   first and increasing it only after timeout/deadman behavior is confirmed.
7. Store observations with the YAML parameters so the experiment can be replayed.

## 8. Result

The final system validates the complete path from a physical joystick to a
Chrono HMMWV simulation. The demo shows that ROS 2 can be used as the message
interface between HID input and a Project Chrono vehicle dynamics model inside a
Docker environment.

The most important implementation decisions are:

- Use Docker device passthrough for joystick access.
- Use X11 forwarding for visualization.
- Keep ROS 2 as the runtime control bus.
- Keep joystick mapping and safety policy in YAML.
- Clear stale or disabled joystick input through timeout/deadman safety output.
- Use primitive visualization to reduce asset-related failures.
- Provide software OpenGL fallback for machines without working container GPU
  acceleration.
