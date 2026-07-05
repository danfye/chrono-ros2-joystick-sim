// Copyright 2026 Chrono ROS 2 Demo Maintainer
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <vector>

namespace chrono_sim_control
{

struct DriverCommand
{
  double steering = 0.0;
  double throttle = 0.0;
  double braking = 0.0;
};

struct JoystickMapperConfig
{
  int steering_axis = 0;
  int throttle_brake_axis = 1;
  int throttle_axis = -1;
  int brake_axis = -1;
  int enable_button = -1;
  int deadman_button = -1;
  double steering_scale = 1.0;
  double throttle_scale = 1.0;
  double brake_scale = 1.0;
  bool invert_steering = false;
  bool invert_throttle = false;
  bool invert_brake = false;
  double deadzone = 0.1;
  double timeout_seconds = 0.5;
  bool safe_brake_on_timeout = true;
  double safe_brake_value = 0.0;
};

class JoystickMapper
{
public:
  explicit JoystickMapper(const JoystickMapperConfig & config = JoystickMapperConfig());

  void ResetSafetyTimer(double now_seconds);
  DriverCommand UpdateFromJoy(
    const std::vector<float> & axes, const std::vector<int> & buttons,
    double now_seconds);
  DriverCommand CommandAt(double now_seconds) const;
  DriverCommand SafeCommand() const;

  bool last_message_valid() const {return last_message_valid_;}
  bool is_enabled() const {return enabled_;}
  bool is_timed_out(double now_seconds) const;

  const JoystickMapperConfig & config() const {return config_;}

private:
  static double Clamp(double value, double min_value, double max_value);
  static double ApplyDeadzone(double value, double deadzone);
  static bool ButtonPressed(const std::vector<int> & buttons, int index, bool * ok);

  double ReadAxis(
    const std::vector<float> & axes, int index, bool invert, double scale,
    bool * ok) const;
  double ReadOptionalAxis(
    const std::vector<float> & axes, int index, bool invert, double scale,
    bool * ok) const;
  DriverCommand MapAxes(const std::vector<float> & axes, bool * ok) const;
  bool EnableButtonsPressed(const std::vector<int> & buttons, bool * ok) const;

  JoystickMapperConfig config_;
  DriverCommand command_;
  double last_joy_time_seconds_ = 0.0;
  double safety_timer_start_seconds_ = 0.0;
  bool have_joy_ = false;
  bool last_message_valid_ = false;
  bool enabled_ = true;
};

}  // namespace chrono_sim_control
