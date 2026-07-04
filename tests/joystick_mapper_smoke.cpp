#include "chrono_sim_control/joystick_mapper.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

bool Near(double actual, double expected) {
    return std::abs(actual - expected) < 1e-6;
}

void Check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

}  // namespace

int main() {
    using chrono_sim_control::JoystickMapper;
    using chrono_sim_control::JoystickMapperConfig;

    JoystickMapperConfig config;
    config.deadzone = 0.1;
    config.timeout_seconds = 0.5;
    config.safe_brake_on_timeout = true;
    config.safe_brake_value = 0.4;

    JoystickMapper mapper(config);
    auto command = mapper.UpdateFromJoy({0.05F, 0.8F}, {}, 1.0);
    Check(mapper.last_message_valid(), "valid joystick message should be accepted");
    Check(Near(command.steering, 0.0), "deadzone should clear small steering input");
    Check(Near(command.throttle, 0.8), "positive combined axis should map to throttle");
    Check(Near(command.braking, 0.0), "positive combined axis should not brake");

    command = mapper.UpdateFromJoy({-0.5F, -0.7F}, {}, 1.1);
    Check(Near(command.steering, -0.5), "steering axis should pass through");
    Check(Near(command.throttle, 0.0), "negative combined axis should not throttle");
    Check(Near(command.braking, 0.7), "negative combined axis should map to braking");

    command = mapper.CommandAt(1.8);
    Check(mapper.is_timed_out(1.8), "mapper should time out stale joystick input");
    Check(Near(command.steering, 0.0), "timeout should clear steering");
    Check(Near(command.throttle, 0.0), "timeout should clear throttle");
    Check(Near(command.braking, 0.4), "timeout should apply configured safe brake");

    std::cout << "joystick mapper smoke ok\n";
    return 0;
}
