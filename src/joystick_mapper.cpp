// Copyright 2026 Chrono ROS 2 Demo Maintainer
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "chrono_sim_control/joystick_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace chrono_sim_control
{

JoystickMapper::JoystickMapper(const JoystickMapperConfig & config)
: config_(config)
{
  config_.deadzone = Clamp(config_.deadzone, 0.0, 0.999);
  config_.timeout_seconds = std::max(0.0, config_.timeout_seconds);
  config_.safe_brake_value = Clamp(config_.safe_brake_value, 0.0, 1.0);
}

void JoystickMapper::ResetSafetyTimer(double now_seconds)
{
  safety_timer_start_seconds_ = now_seconds;
}

DriverCommand JoystickMapper::UpdateFromJoy(
  const std::vector<float> & axes,
  const std::vector<int> & buttons,
  double now_seconds)
{
  bool buttons_ok = true;
  enabled_ = EnableButtonsPressed(buttons, &buttons_ok);

  if (!buttons_ok) {
    last_message_valid_ = false;
    command_ = SafeCommand();
    return CommandAt(now_seconds);
  }

  if (!enabled_) {
    last_joy_time_seconds_ = now_seconds;
    have_joy_ = true;
    command_ = DriverCommand();
    last_message_valid_ = true;
    return CommandAt(now_seconds);
  }

  bool ok = true;
  const auto mapped = MapAxes(axes, &ok);
  last_message_valid_ = ok;

  if (ok) {
    last_joy_time_seconds_ = now_seconds;
    have_joy_ = true;
    command_ = mapped;
  } else {
    command_ = SafeCommand();
  }

  return CommandAt(now_seconds);
}

DriverCommand JoystickMapper::CommandAt(double now_seconds) const
{
  if (!enabled_ || is_timed_out(now_seconds)) {
    return SafeCommand();
  }

  return command_;
}

DriverCommand JoystickMapper::SafeCommand() const
{
  DriverCommand safe;
  if (config_.safe_brake_on_timeout) {
    safe.braking = config_.safe_brake_value;
  }
  return safe;
}

bool JoystickMapper::is_timed_out(double now_seconds) const
{
  const double last_input_time = have_joy_ ? last_joy_time_seconds_ : safety_timer_start_seconds_;

  return config_.timeout_seconds > 0.0 && (now_seconds - last_input_time) > config_.timeout_seconds;
}

double JoystickMapper::Clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double JoystickMapper::ApplyDeadzone(double value, double deadzone)
{
  return std::abs(value) < deadzone ? 0.0 : value;
}

bool JoystickMapper::ButtonPressed(const std::vector<int> & buttons, int index, bool * ok)
{
  if (index < 0) {
    return true;
  }

  const auto button_index = static_cast<std::size_t>(index);
  if (button_index >= buttons.size()) {
    *ok = false;
    return false;
  }

  return button_index < buttons.size() && buttons[button_index] != 0;
}

double JoystickMapper::ReadAxis(
  const std::vector<float> & axes,
  int index,
  bool invert,
  double scale,
  bool * ok) const
{
  if (index < 0 || static_cast<std::size_t>(index) >= axes.size()) {
    *ok = false;
    return 0.0;
  }

  const double raw =
    ApplyDeadzone(
    Clamp(
      static_cast<double>(axes[static_cast<std::size_t>(index)]), -1.0,
      1.0), config_.deadzone);
  const double sign = invert ? -1.0 : 1.0;
  return raw * sign * scale;
}

double JoystickMapper::ReadOptionalAxis(
  const std::vector<float> & axes,
  int index,
  bool invert,
  double scale,
  bool * ok) const
{
  if (index < 0) {
    return 0.0;
  }

  return ReadAxis(axes, index, invert, scale, ok);
}

DriverCommand JoystickMapper::MapAxes(const std::vector<float> & axes, bool * ok) const
{
  DriverCommand mapped;
  mapped.steering = ReadAxis(
    axes, config_.steering_axis, config_.invert_steering,
    config_.steering_scale, ok);

  const bool independent_pedals = config_.throttle_axis >= 0 || config_.brake_axis >= 0;
  if (independent_pedals) {
    mapped.throttle =
      std::max(
      0.0,
      ReadOptionalAxis(
        axes, config_.throttle_axis, config_.invert_throttle,
        config_.throttle_scale, ok));
    mapped.braking =
      std::max(
      0.0,
      ReadOptionalAxis(axes, config_.brake_axis, config_.invert_brake, config_.brake_scale, ok));
  } else {
    const bool invert_combined_axis = config_.invert_throttle || config_.invert_brake;
    const double throttle_brake = ReadAxis(
      axes, config_.throttle_brake_axis, invert_combined_axis,
      1.0, ok);
    if (throttle_brake > 0.0) {
      mapped.throttle = throttle_brake * config_.throttle_scale;
    } else {
      mapped.braking = -throttle_brake * config_.brake_scale;
    }
  }

  mapped.steering = Clamp(mapped.steering, -1.0, 1.0);
  mapped.throttle = Clamp(mapped.throttle, 0.0, 1.0);
  mapped.braking = Clamp(mapped.braking, 0.0, 1.0);
  return mapped;
}

bool JoystickMapper::EnableButtonsPressed(const std::vector<int> & buttons, bool * ok) const
{
  const bool enable_pressed = ButtonPressed(buttons, config_.enable_button, ok);
  const bool deadman_pressed = ButtonPressed(buttons, config_.deadman_button, ok);
  return enable_pressed && deadman_pressed;
}

}  // namespace chrono_sim_control
