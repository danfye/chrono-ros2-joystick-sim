// Copyright 2026 Chrono ROS 2 Demo Maintainer
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "chrono_sim_control/joystick_mapper.hpp"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using chrono::ChBody;
using chrono::ChCollisionShapeBox;
using chrono::ChColor;
using chrono::ChContactMaterialNSC;
using chrono::ChContactMethod;
using chrono::ChCoordsys;
using chrono::ChFrame;
using chrono::ChSystem;
using chrono::ChVector3d;
using chrono::ChVisualShapeBox;
using chrono::QUNIT;
using chrono::SetChronoDataPath;
using chrono::vehicle::ChTerrain;
using chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht;
using chrono::vehicle::DriverInputs;
using chrono::vehicle::EngineModelType;
using chrono::vehicle::TireModelType;
using chrono::vehicle::VisualizationType;
using chrono_sim_control::JoystickMapper;
using chrono_sim_control::JoystickMapperConfig;

class JoySubscriberNode : public rclcpp::Node
{
public:
  JoySubscriberNode()
  : Node("chrono_joy_driver"), mapper_(LoadMapperConfig())
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    step_size_ = declare_parameter<double>("step_size", 0.002);
    headless_ = declare_parameter<bool>("headless", false);
    max_runtime_seconds_ = declare_parameter<double>("max_runtime_seconds", 0.0);
    chrono_data_path_ = declare_parameter<std::string>(
      "chrono_data_path",
      "/usr/local/share/chrono/data/");
    mapper_.ResetSafetyTimer(get_clock()->now().seconds());

    if (step_size_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid step_size %.6f; using 0.002", step_size_);
      step_size_ = 0.002;
    }
    if (max_runtime_seconds_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "Invalid max_runtime_seconds %.3f; running without a time limit",
        max_runtime_seconds_);
      max_runtime_seconds_ = 0.0;
    }

    subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      10,
      std::bind(&JoySubscriberNode::joy_callback, this, std::placeholders::_1));
    safety_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&JoySubscriberNode::safety_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Listening on %s: steering_axis=%d throttle_brake_axis=%d throttle_axis=%d brake_axis=%d "
      "deadzone=%.2f timeout=%.2fs safe_brake=%s/%.2f step_size=%.4f",
      joy_topic_.c_str(),
      mapper_.config().steering_axis,
      mapper_.config().throttle_brake_axis,
      mapper_.config().throttle_axis,
      mapper_.config().brake_axis,
      mapper_.config().deadzone,
      mapper_.config().timeout_seconds,
      mapper_.config().safe_brake_on_timeout ? "on" : "off",
      mapper_.config().safe_brake_value,
      step_size_);

    RCLCPP_INFO(
      get_logger(),
      "Chrono runtime mode: %s%s",
      headless_ ? "headless" : "visual",
      max_runtime_seconds_ > 0.0 ? ", finite runtime" : "");
  }

  DriverInputs GetDriverInputs(double now_seconds) const
  {
    const auto command = mapper_.CommandAt(now_seconds);
    DriverInputs inputs;
    inputs.m_steering = command.steering;
    inputs.m_throttle = command.throttle;
    inputs.m_braking = command.braking;
    return inputs;
  }

  double GetStepSize() const {return step_size_;}
  bool IsHeadless() const {return headless_;}
  double GetMaxRuntimeSeconds() const {return max_runtime_seconds_;}
  const std::string & GetChronoDataPath() const {return chrono_data_path_;}

private:
  JoystickMapperConfig LoadMapperConfig()
  {
    JoystickMapperConfig config;
    config.steering_axis = declare_parameter<int>("steering_axis", config.steering_axis);
    config.throttle_brake_axis = declare_parameter<int>(
      "throttle_brake_axis",
      config.throttle_brake_axis);
    config.throttle_axis = declare_parameter<int>("throttle_axis", config.throttle_axis);
    config.brake_axis = declare_parameter<int>("brake_axis", config.brake_axis);
    config.enable_button = declare_parameter<int>("enable_button", config.enable_button);
    config.deadman_button = declare_parameter<int>("deadman_button", config.deadman_button);
    config.steering_scale = declare_parameter<double>("steering_scale", config.steering_scale);
    config.throttle_scale = declare_parameter<double>("throttle_scale", config.throttle_scale);
    config.brake_scale = declare_parameter<double>("brake_scale", config.brake_scale);
    config.invert_steering = declare_parameter<bool>("invert_steering", config.invert_steering);
    config.invert_throttle = declare_parameter<bool>("invert_throttle", config.invert_throttle);
    config.invert_brake = declare_parameter<bool>("invert_brake", config.invert_brake);
    config.deadzone = declare_parameter<double>("deadzone", config.deadzone);
    config.timeout_seconds = declare_parameter<double>(
      "joystick_timeout_seconds",
      config.timeout_seconds);
    config.safe_brake_on_timeout = declare_parameter<bool>(
      "safe_brake_on_timeout",
      config.safe_brake_on_timeout);
    config.safe_brake_value =
      declare_parameter<double>("safe_brake_value", config.safe_brake_value);
    return config;
  }

  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    const double now_seconds = get_clock()->now().seconds();
    const auto command = mapper_.UpdateFromJoy(msg->axes, msg->buttons, now_seconds);
    if (!mapper_.last_message_valid()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Joy message does not match configured axes/buttons");
      return;
    }

    if (!mapper_.is_enabled()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 1000, "Joystick deadman/enable button is not pressed");
    }

    if (std::abs(command.steering) > 0.0 || command.throttle > 0.0 || command.braking > 0.0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "Joy Control: steer=%.2f throttle=%.2f brake=%.2f",
        command.steering,
        command.throttle,
        command.braking);
    }
  }

  void safety_timer_callback()
  {
    if (mapper_.is_timed_out(get_clock()->now().seconds())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "No /joy input for %.2f seconds; applying safe command",
        mapper_.config().timeout_seconds);
    }
  }

  std::string joy_topic_;
  double step_size_ = 0.002;
  bool headless_ = false;
  double max_runtime_seconds_ = 0.0;
  std::string chrono_data_path_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  JoystickMapper mapper_;
};

class SimpleFlatTerrain : public ChTerrain
{
public:
  explicit SimpleFlatTerrain(ChSystem * system)
  {
    material_ = chrono_types::make_shared<ChContactMaterialNSC>();
    material_->SetFriction(0.8f);

    terrain_body = chrono_types::make_shared<ChBody>();
    terrain_body->SetFixed(true);

    auto shape = chrono_types::make_shared<ChCollisionShapeBox>(material_, 200, 200, 1);
    terrain_body->AddCollisionShape(shape, ChFrame<>(ChVector3d(0, 0, -1), QUNIT));
    terrain_body->EnableCollision(true);

    auto visual_asset = chrono_types::make_shared<ChVisualShapeBox>(200, 200, 1);
    visual_asset->SetColor(ChColor(0.1f, 0.1f, 0.1f));
    terrain_body->AddVisualShape(visual_asset, ChFrame<>(ChVector3d(0, 0, -1)));

    system->Add(terrain_body);
  }

  double GetHeight(const ChVector3d & loc) const override
  {
    (void)loc;
    return 0.0;
  }

  ChVector3d GetNormal(const ChVector3d & loc) const override
  {
    (void)loc;
    return ChVector3d(0, 0, 1);
  }

  std::shared_ptr<ChBody> terrain_body;

private:
  std::shared_ptr<ChContactMaterialNSC> material_;
};

std::string EnsureTrailingSlash(const std::string & path)
{
  if (path.empty() || path.back() == '/') {
    return path;
  }

  return path + "/";
}

bool HasReachedMaxRuntime(double current_time, double max_runtime_seconds)
{
  return max_runtime_seconds > 0.0 && current_time >= max_runtime_seconds;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<JoySubscriberNode>();

  const auto chrono_data_path = EnsureTrailingSlash(ros_node->GetChronoDataPath());
  SetChronoDataPath(chrono_data_path);
  chrono::vehicle::SetDataPath(chrono_data_path + "vehicle/");

  chrono::vehicle::hmmwv::HMMWV_Full my_hmmwv;
  my_hmmwv.SetContactMethod(ChContactMethod::NSC);
  my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 1.0), QUNIT));
  my_hmmwv.SetEngineType(EngineModelType::SIMPLE);
  my_hmmwv.SetTireType(TireModelType::TMEASY);
  my_hmmwv.Initialize();

  my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
  my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);
  my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
  my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
  my_hmmwv.SetWheelVisualizationType(VisualizationType::PRIMITIVES);

  SimpleFlatTerrain custom_terrain(my_hmmwv.GetSystem());

  const double step_size = ros_node->GetStepSize();
  const double max_runtime_seconds = ros_node->GetMaxRuntimeSeconds();
  if (ros_node->IsHeadless()) {
    RCLCPP_INFO(
      ros_node->get_logger(),
      "Starting headless Chrono simulation%s",
      max_runtime_seconds > 0.0 ? " with a finite runtime" : "");

    while (rclcpp::ok()) {
      rclcpp::spin_some(ros_node);

      const double time = my_hmmwv.GetSystem()->GetChTime();
      if (HasReachedMaxRuntime(time, max_runtime_seconds)) {
        break;
      }

      const auto driver_inputs = ros_node->GetDriverInputs(ros_node->get_clock()->now().seconds());
      my_hmmwv.Synchronize(time, driver_inputs, custom_terrain);
      my_hmmwv.Advance(step_size);
    }

    rclcpp::shutdown();
    return 0;
  }

  auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
  vis->SetWindowTitle("Chrono ROS 2 HMMWV Control");
  vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
  vis->Initialize();
  vis->AttachVehicle(&my_hmmwv.GetVehicle());
  vis->AddLightDirectional();
  vis->AddSkyBox();
  vis->AddGrid(
    1.0,
    1.0,
    200,
    200,
    ChCoordsys<>(ChVector3d(0, 0, 0.02), QUNIT),
    ChColor(0.5f, 0.5f, 0.5f));

  vis->EnableCollisionShapeDrawing(false);
  vis->EnableBodyFrameDrawing(false);

  while (vis->Run() && rclcpp::ok()) {
    rclcpp::spin_some(ros_node);

    const double time = my_hmmwv.GetSystem()->GetChTime();
    if (HasReachedMaxRuntime(time, max_runtime_seconds)) {
      break;
    }

    const auto driver_inputs = ros_node->GetDriverInputs(ros_node->get_clock()->now().seconds());

    my_hmmwv.Synchronize(time, driver_inputs, custom_terrain);
    vis->Synchronize(time, driver_inputs);

    my_hmmwv.Advance(step_size);
    vis->Advance(step_size);

    vis->BeginScene();
    vis->Render();
    vis->EndScene();
  }

  rclcpp::shutdown();
  return 0;
}
