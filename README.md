# Chrono ROS 2 Joystick Simulation

This repository is a reproducible ROS 2 Humble package for controlling a
Project Chrono HMMWV vehicle simulation with a physical joystick.

```text
Joystick -> Linux /dev/input -> Docker -> ROS 2 joy_node -> /joy
         -> chrono_joy_driver -> Chrono HMMWV vehicle simulation
```

The package name is `chrono_sim_control`. The main executable is
`chrono_joy_driver`.

## Features

- Project Chrono `HMMWV_Full` vehicle dynamics simulation.
- ROS 2 Humble joystick integration through `sensor_msgs/msg/Joy`.
- Standard `ament_cmake` package with launch and YAML configuration files.
- Docker runtime for Ubuntu 22.04, ROS 2 Humble, Project Chrono 9.0, X11, and
  `/dev/input` joystick passthrough.
- Primitive vehicle visualization and flat terrain for a robust container demo.
- Manual `/joy` publishing path for validation without a physical joystick.
- Configurable joystick mapping for different controllers without changing C++.
- Joystick timeout and optional deadman-button safety behavior for repeatable
  experiments.

## Repository Structure

```text
.
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── chrono_joy_driver.yaml
│   └── profiles/
│       ├── combined_axis_coast.yaml
│       └── split_triggers_deadman_brake.yaml
├── launch/
│   └── joystick_hmmwv.launch.py
├── src/
│   └── simplified_driver.cpp
├── docker/
│   ├── Dockerfile
│   ├── run-container.md
│   └── run.sh
├── docs/
│   └── implementation-report.md
└── LICENSE
```

## Joystick Control Configuration

Default node parameters live in `config/chrono_joy_driver.yaml`. The default
mapping keeps the original demo behavior:

- `axes[0]` steers the vehicle.
- `axes[1] > 0` applies throttle.
- `axes[1] < 0` applies braking.
- Small input noise inside `deadzone` is treated as zero.

```yaml
chrono_joy_driver:
  ros__parameters:
    joy_topic: /joy
    steering_axis: 0
    invert_steering: false
    steering_scale: 1.0
    throttle_brake_axis: 1
    invert_throttle: false
    invert_brake: false
    throttle_axis: -1
    brake_axis: -1
    throttle_scale: 1.0
    brake_scale: 1.0
    enable_button: -1
    deadman_button: -1
    deadzone: 0.1
    joystick_timeout_seconds: 0.5
    safe_brake_on_timeout: true
    safe_brake_value: 0.0
    step_size: 0.002
    chrono_data_path: /usr/local/share/chrono/data/
```

### Parameter Reference

| Parameter | Default | Purpose |
| --- | --- | --- |
| `joy_topic` | `/joy` | ROS 2 topic that carries `sensor_msgs/msg/Joy`. |
| `steering_axis` | `0` | Axis index used for steering. |
| `invert_steering` | `false` | Reverses steering sign for controllers with opposite axis orientation. |
| `steering_scale` | `1.0` | Multiplies steering before clamping to `[-1, 1]`. |
| `throttle_brake_axis` | `1` | Combined throttle/brake axis used when both split-pedal axes stay at `-1`. |
| `invert_throttle` | `false` | Reverses the throttle sign. |
| `invert_brake` | `false` | Reverses the brake sign. |
| `throttle_axis` | `-1` | Optional independent throttle axis. Set to `>= 0` to enable split-pedal mode. |
| `brake_axis` | `-1` | Optional independent brake axis. Set to `>= 0` to enable split-pedal mode. |
| `throttle_scale` | `1.0` | Multiplies throttle before clamping to `[0, 1]`. |
| `brake_scale` | `1.0` | Multiplies braking before clamping to `[0, 1]`. |
| `enable_button` | `-1` | Optional button that must be pressed before input is applied. |
| `deadman_button` | `-1` | Optional second button that must also be pressed before input is applied. |
| `deadzone` | `0.1` | Suppresses small joystick noise around zero. |
| `joystick_timeout_seconds` | `0.5` | Maximum allowed time since the last valid `/joy` message before safety output is used. |
| `safe_brake_on_timeout` | `true` | Applies `safe_brake_value` when timeout or deadman disables input. |
| `safe_brake_value` | `0.0` | Brake command used in timeout or deadman-inactive state. Use `0.0` for zero-input coast or a positive value for safety braking. |
| `step_size` | `0.002` | Chrono simulation step size in seconds. |
| `chrono_data_path` | `/usr/local/share/chrono/data/` | Project Chrono data directory. |
| `headless` | `false` | Runs the Chrono driver without opening an Irrlicht window. Useful for automated local smoke tests. |
| `max_runtime_seconds` | `0.0` | Stops the driver after this simulation time. `0.0` means run until interrupted or the window closes. |

For most controllers, first run `ros2 topic echo /joy`, move each stick/trigger,
and record the axis and button indices. Then update only the YAML file and
restart the launch command.

Two ready-to-edit profiles are included:

- `config/profiles/combined_axis_coast.yaml`: same signed-axis behavior as the
  default demo, with timeout/deadman clearing stale input to zero.
- `config/profiles/split_triggers_deadman_brake.yaml`: split trigger example
  with enable/deadman buttons and active safety braking.

Use a profile without changing the launch path:

```bash
ros2 launch chrono_sim_control joystick_hmmwv.launch.py \
  config_file:=/workspace/chrono_sim_control/config/profiles/split_triggers_deadman_brake.yaml
```

### Safety Behavior

The joystick driver should never keep stale throttle after the operator stops
sending input. The safety policy is:

- If no `/joy` message arrives for more than `joystick_timeout_seconds`, steering
  and throttle are cleared and braking is set to `safe_brake_value`.
- If `enable_button` or `deadman_button` is configured, mapped joystick input is
  applied only while all configured buttons are pressed.
- If a button is released, steering and throttle are cleared and braking is set
  to `safe_brake_value`.
- If a configured axis or button index is missing from a `/joy` message, the
  driver treats that message as invalid, does not refresh the timeout timer, and
  keeps the vehicle in the safety output state.

For bench tests, `safe_brake_value: 0.0` is gentle because it commands zero
input. For vehicle-stop experiments, use a positive value such as `0.4` to make
timeout and deadman release visibly brake the HMMWV.

## Quick Smoke Test Without ROS 2

The joystick mapping core is intentionally separated from ROS 2 and Project
Chrono dependencies. On a normal Linux or macOS machine with a C++17 compiler,
run:

```bash
./scripts/run_mapper_smoke.sh
```

Expected output:

```text
joystick mapper smoke ok
```

This smoke test verifies deadzone handling, throttle/brake mapping, steering,
and timeout safety behavior. It is not a substitute for the full ROS 2 + Chrono
simulation, but it gives reviewers a fast way to confirm the core control logic
builds and runs before setting up the heavier container environment.

## Local ROS 2 Smoke Test With Cached Chrono

If Chrono is already installed in a persistent local directory, run the ROS 2
build, tests, and headless driver startup with:

```bash
./scripts/run_ros2_smoke.sh
```

By default the script uses:

- Docker image: `chrono_ros2_deps:humble`
- verification root: `/tmp/chrono_ros2_verify`
- Chrono install: `/tmp/chrono_ros2_verify/chrono_install`

Override those paths when needed:

```bash
IMAGE_NAME=chrono_ros2_deps:humble \
VERIFY_ROOT=/tmp/chrono_ros2_verify \
CHRONO_INSTALL=/tmp/chrono_ros2_verify/chrono_install \
HEADLESS_RUNTIME_SECONDS=0.05 \
./scripts/run_ros2_smoke.sh
```

The smoke test intentionally uses `headless:=true`, so it does not require X11,
GPU passthrough, or a physical joystick. It also launches with
`start_joy_node:=false`, because the smoke test publishes no physical joystick
events.

## Build the Docker Image

Build the image from the repository root:

```bash
docker build -t chrono_ros2_joystick:humble -f docker/Dockerfile .
```

The Dockerfile builds Project Chrono from `release/9.0` with the Vehicle,
Irrlicht, and Postprocess modules enabled. This can take a while on first build.

## Start the Container

On the Linux host, allow local X11 clients and start the container:

```bash
chmod +x docker/run.sh
./docker/run.sh
```

The script mounts:

- this repository at `/workspace/chrono_sim_control`
- `/dev/input` for joystick/HID devices
- `/tmp/.X11-unix` and `DISPLAY` for X11 visualization
- `/dev/dri` for direct rendering when available

For a different image name:

```bash
IMAGE_NAME=chrono_ros2_joystick:humble ./docker/run.sh
```

## Build the ROS 2 Package

Inside the container:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select chrono_sim_control
source install/setup.bash
```

Run the mapper unit tests:

```bash
colcon test --packages-select chrono_sim_control
colcon test-result --verbose
```

The tests cover deadzone, axis bounds, combined throttle/brake, split pedals,
axis inversion/scaling, timeout from startup and after valid input, deadman
buttons, and missing configured buttons.

## Run the Simulation

Start the joystick node and Chrono simulation together:

```bash
ros2 launch chrono_sim_control joystick_hmmwv.launch.py
```

The launch file starts:

- `joy_node` from the ROS 2 `joy` package
- `chrono_joy_driver` from this package

The Chrono window should open through X11 and show the HMMWV on a flat grid.

For a headless launch check without X11 or a joystick:

```bash
ros2 launch chrono_sim_control joystick_hmmwv.launch.py \
  start_joy_node:=false \
  headless:=true \
  max_runtime_seconds:=0.05
```

## Validate Without a Physical Joystick

This path does not use `joy_node`; it publishes synthetic messages directly to
the same `/joy` topic. In one terminal, start only the Chrono driver:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run chrono_sim_control chrono_joy_driver
```

In another terminal inside the same container, publish synthetic joystick input:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /joy sensor_msgs/msg/Joy \
  "{axes: [0.5, 0.8], buttons: []}" -r 10
```

Expected behavior:

- `axes[0] = 0.5` steers right.
- `axes[1] = 0.8` applies throttle.
- `axes[1] = -0.8` applies braking.
- Stopping the publisher should clear stale joystick input after
  `joystick_timeout_seconds`.

Example braking command:

```bash
ros2 topic pub /joy sensor_msgs/msg/Joy \
  "{axes: [0.0, -0.8], buttons: []}" -r 10
```

Example timeout check:

```bash
ros2 topic pub --once /joy sensor_msgs/msg/Joy \
  "{axes: [0.3, 0.7], buttons: []}"
```

The vehicle may briefly receive the command, then the driver should return to
zero input or safety braking after the configured timeout.

Example deadman check with `enable_button: 4` and `deadman_button: 5`:

```bash
ros2 topic pub /joy sensor_msgs/msg/Joy \
  "{axes: [0.3, 0.7], buttons: [0, 0, 0, 0, 1, 1]}" -r 10
```

Changing either required button value from `1` to `0` should force the safety output.

## Validate With a Physical Joystick

Inside the container, check that Linux input devices are visible:

```bash
ls /dev/input
```

Start the ROS 2 joystick node:

```bash
source /opt/ros/humble/setup.bash
ros2 run joy joy_node
```

Move the joystick and confirm `/joy` messages arrive:

```bash
ros2 topic echo /joy
```

Then run the full launch command:

```bash
source install/setup.bash
ros2 launch chrono_sim_control joystick_hmmwv.launch.py
```

## Recommended Experiment Flow

1. Build and launch through Docker first so ROS 2 Humble, Chrono 9.0, X11, and
   `/dev/input` passthrough are controlled.
2. Confirm `/joy` messages with `ros2 topic echo /joy` before starting a driving
   run.
3. Calibrate the controller by recording steering, throttle, brake, and optional
   deadman indices; update `config/chrono_joy_driver.yaml` rather than editing
   C++.
4. Validate synthetic `/joy` commands for steering, throttle, braking, timeout,
   and deadman release.
5. Run the physical joystick trial at low speed with the default
   `safe_brake_value: 0.0`; switch to a positive value or the
   `split_triggers_deadman_brake.yaml` profile only after confirming the
   timeout/deadman behavior is stable.
6. Record the YAML file, Docker image tag, and launch command with each
   experiment so the input mapping and safety behavior are reproducible.

## OpenGL Software Fallback

If the container reports an OpenGL driver error such as:

```text
libGL error: failed to load driver: nouveau
```

use Mesa software rendering before launching the simulation:

```bash
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch chrono_sim_control joystick_hmmwv.launch.py
```

Software rendering is slower, but it is useful for validating the ROS 2 and
Chrono control path on machines without working container GPU acceleration.

## Native Ubuntu Build

The Docker path is the recommended reproducible setup. For a native Ubuntu
22.04 machine, install ROS 2 Humble, install the `joy` package, build Chrono 9.0
with Vehicle and Irrlicht enabled, then run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select chrono_sim_control
source install/setup.bash
ros2 launch chrono_sim_control joystick_hmmwv.launch.py
```

## Troubleshooting

### `joy_node` starts but `/joy` has no output

Make sure the container can read the host input devices:

```bash
ls /dev/input
ros2 topic echo /joy
```

Modern controllers often appear under `/dev/input/event*`, not only
`/dev/input/js0`. The default `docker/run.sh` mounts the full `/dev/input`
directory and uses `--privileged` for a simple research setup.

### Vehicle mesh assets are missing

The simulation intentionally uses Chrono primitive visualization for chassis,
tires, suspension, steering, and wheels. This avoids relying on external mesh
assets inside the container.

### Chrono data files are installed somewhere else

Override the data path parameter:

```bash
ros2 launch chrono_sim_control joystick_hmmwv.launch.py \
  config_file:=/workspace/chrono_sim_control/config/chrono_joy_driver.yaml
```

or edit `config/chrono_joy_driver.yaml` and rebuild/source the workspace if the
file is installed into `install/`.

## License

This repository is prepared with the MIT License. Check the licenses of ROS 2,
Project Chrono, Irrlicht, and any assets or containers used in your deployment.
