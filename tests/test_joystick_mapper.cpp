// Copyright 2026 Chrono ROS 2 Demo Maintainer
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <vector>

#include "chrono_sim_control/joystick_mapper.hpp"

using chrono_sim_control::DriverCommand;
using chrono_sim_control::JoystickMapper;
using chrono_sim_control::JoystickMapperConfig;

namespace
{

constexpr double kCommandTolerance = 1e-6;

void ExpectCommandNear(
  const DriverCommand & command,
  double steering,
  double throttle,
  double braking)
{
  EXPECT_NEAR(command.steering, steering, kCommandTolerance);
  EXPECT_NEAR(command.throttle, throttle, kCommandTolerance);
  EXPECT_NEAR(command.braking, braking, kCommandTolerance);
}

}  // namespace

TEST(JoystickMapperTest, AppliesDeadzoneAndClampsAxes) {
  JoystickMapperConfig config;
  config.deadzone = 0.1;
  config.safe_brake_on_timeout = false;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.05F, 2.0F}, {}, 1.0);

  EXPECT_TRUE(mapper.last_message_valid());
  ExpectCommandNear(command, 0.0, 1.0, 0.0);
}

TEST(JoystickMapperTest, AppliesSafeCommandWhenAxisIsOutOfRange) {
  JoystickMapperConfig config;
  config.safe_brake_on_timeout = true;
  config.safe_brake_value = 0.6;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.25F, 0.5F}, {}, 1.0);
  ExpectCommandNear(command, 0.25, 0.5, 0.0);

  command = mapper.UpdateFromJoy({0.0F}, {}, 1.1);

  EXPECT_FALSE(mapper.last_message_valid());
  ExpectCommandNear(command, 0.0, 0.0, 0.6);
}

TEST(JoystickMapperTest, InvalidAxisDoesNotRefreshTimeout) {
  JoystickMapperConfig config;
  config.timeout_seconds = 0.5;
  config.safe_brake_on_timeout = true;
  config.safe_brake_value = 0.6;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.25F, 0.5F}, {}, 1.0);
  ExpectCommandNear(command, 0.25, 0.5, 0.0);

  command = mapper.UpdateFromJoy({0.0F}, {}, 1.4);
  EXPECT_FALSE(mapper.last_message_valid());
  ExpectCommandNear(command, 0.0, 0.0, 0.6);

  EXPECT_TRUE(mapper.is_timed_out(1.6));
}

TEST(JoystickMapperTest, MapsPositiveAndNegativeCombinedThrottleBrakeAxis) {
  JoystickMapperConfig config;
  config.safe_brake_on_timeout = false;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.0F, 0.7F}, {}, 1.0);
  ExpectCommandNear(command, 0.0, 0.7, 0.0);

  command = mapper.UpdateFromJoy({0.0F, -0.4F}, {}, 1.1);
  ExpectCommandNear(command, 0.0, 0.0, 0.4);
}

TEST(JoystickMapperTest, MapsIndependentThrottleAndBrakeAxes) {
  JoystickMapperConfig config;
  config.throttle_axis = 2;
  config.brake_axis = 3;
  config.safe_brake_on_timeout = false;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.2F, 0.0F, 0.6F, 0.3F}, {}, 1.0);

  ExpectCommandNear(command, 0.2, 0.6, 0.3);
}

TEST(JoystickMapperTest, AllowsSingleIndependentPedalAxis) {
  JoystickMapperConfig config;
  config.throttle_axis = 2;
  config.brake_axis = -1;
  config.safe_brake_on_timeout = false;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.2F, 0.0F, 0.6F}, {}, 1.0);

  EXPECT_TRUE(mapper.last_message_valid());
  ExpectCommandNear(command, 0.2, 0.6, 0.0);
}

TEST(JoystickMapperTest, SupportsAxisInversionAndScaling) {
  JoystickMapperConfig config;
  config.invert_steering = true;
  config.steering_scale = 0.5;
  config.throttle_scale = 0.25;
  config.brake_scale = 0.5;
  config.safe_brake_on_timeout = false;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.8F, 0.8F}, {}, 1.0);
  ExpectCommandNear(command, -0.4, 0.2, 0.0);

  command = mapper.UpdateFromJoy({0.0F, -0.8F}, {}, 1.1);
  ExpectCommandNear(command, 0.0, 0.0, 0.4);
}

TEST(JoystickMapperTest, ReversesCombinedThrottleBrakeAxisWithEitherInvertFlag) {
  JoystickMapperConfig config;
  config.invert_throttle = true;
  config.safe_brake_on_timeout = false;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.0F, -0.8F}, {}, 1.0);
  ExpectCommandNear(command, 0.0, 0.8, 0.0);

  config.invert_throttle = false;
  config.invert_brake = true;
  JoystickMapper brake_mapper(config);

  command = brake_mapper.UpdateFromJoy({0.0F, 0.8F}, {}, 1.0);
  ExpectCommandNear(command, 0.0, 0.0, 0.8);
}

TEST(JoystickMapperTest, ReturnsSafeCommandAfterTimeout) {
  JoystickMapperConfig config;
  config.timeout_seconds = 0.5;
  config.safe_brake_on_timeout = true;
  config.safe_brake_value = 0.75;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.2F, 0.8F}, {}, 1.0);
  ExpectCommandNear(command, 0.2, 0.8, 0.0);

  command = mapper.CommandAt(1.6);

  EXPECT_TRUE(mapper.is_timed_out(1.6));
  ExpectCommandNear(command, 0.0, 0.0, 0.75);
}

TEST(JoystickMapperTest, TimesOutFromStartupWithoutJoyMessages) {
  JoystickMapperConfig config;
  config.timeout_seconds = 0.5;
  config.safe_brake_on_timeout = true;
  config.safe_brake_value = 0.4;
  JoystickMapper mapper(config);
  mapper.ResetSafetyTimer(1.0);

  EXPECT_FALSE(mapper.is_timed_out(1.4));
  EXPECT_TRUE(mapper.is_timed_out(1.6));
  ExpectCommandNear(mapper.CommandAt(1.6), 0.0, 0.0, 0.4);
}

TEST(JoystickMapperTest, RequiresDeadmanButtonWhenConfigured) {
  JoystickMapperConfig config;
  config.deadman_button = 1;
  config.safe_brake_on_timeout = false;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.2F, 0.8F}, {0, 0}, 1.0);
  EXPECT_FALSE(mapper.is_enabled());
  ExpectCommandNear(command, 0.0, 0.0, 0.0);

  command = mapper.UpdateFromJoy({0.2F, 0.8F}, {0, 1}, 1.1);
  EXPECT_TRUE(mapper.is_enabled());
  ExpectCommandNear(command, 0.2, 0.8, 0.0);
}

TEST(JoystickMapperTest, RequiresEnableAndDeadmanButtonsTogether) {
  JoystickMapperConfig config;
  config.enable_button = 1;
  config.deadman_button = 2;
  config.safe_brake_on_timeout = false;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.2F, 0.8F}, {0, 1, 0}, 1.0);
  EXPECT_FALSE(mapper.is_enabled());
  ExpectCommandNear(command, 0.0, 0.0, 0.0);

  command = mapper.UpdateFromJoy({0.2F, 0.8F}, {0, 1, 1}, 1.1);
  EXPECT_TRUE(mapper.is_enabled());
  ExpectCommandNear(command, 0.2, 0.8, 0.0);
}

TEST(JoystickMapperTest, MissingConfiguredButtonIsInvalidAndDoesNotRefreshTimeout) {
  JoystickMapperConfig config;
  config.deadman_button = 2;
  config.timeout_seconds = 0.5;
  config.safe_brake_on_timeout = true;
  config.safe_brake_value = 0.4;
  JoystickMapper mapper(config);

  auto command = mapper.UpdateFromJoy({0.2F, 0.8F}, {0, 0, 1}, 1.0);
  ExpectCommandNear(command, 0.2, 0.8, 0.0);

  command = mapper.UpdateFromJoy({0.2F, 0.8F}, {0}, 1.4);

  EXPECT_FALSE(mapper.last_message_valid());
  EXPECT_FALSE(mapper.is_enabled());
  ExpectCommandNear(command, 0.0, 0.0, 0.4);
  EXPECT_TRUE(mapper.is_timed_out(1.6));
}
