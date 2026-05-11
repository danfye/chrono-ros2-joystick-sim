#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace chrono;
using namespace chrono::vehicle;

double g_throttle = 0;
double g_steering = 0;
double g_braking = 0;

class JoySubscriberNode : public rclcpp::Node {
public:
    JoySubscriberNode() : Node("chrono_joy_driver") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoySubscriberNode::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) const {
        if (msg->axes.size() <= 1) {
            return;
        }

        if (std::abs(msg->axes[0]) > 0.1 || std::abs(msg->axes[1]) > 0.1) {
            RCLCPP_INFO(
                this->get_logger(),
                "Joy Control: Steer=%.2f, ThrottleAxis=%.2f",
                msg->axes[0],
                msg->axes[1]);
        }

        g_steering = msg->axes[0];
        double val = msg->axes[1];
        if (val > 0) {
            g_throttle = val;
            g_braking = 0;
        } else {
            g_throttle = 0;
            g_braking = -val;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

class SimpleFlatTerrain : public ChTerrain {
private:
    std::shared_ptr<ChContactMaterialNSC> m_material;

public:
    double GetHeight(const ChVector3d& loc) const override { return 0.0; }
    ChVector3d GetNormal(const ChVector3d& loc) const override { return ChVector3d(0, 0, 1); }

    std::shared_ptr<ChBody> terrain_body;

    SimpleFlatTerrain(ChSystem* system) {
        m_material = chrono_types::make_shared<ChContactMaterialNSC>();
        m_material->SetFriction(0.8f);

        terrain_body = chrono_types::make_shared<ChBody>();
        terrain_body->SetFixed(true);

        auto shape = chrono_types::make_shared<ChCollisionShapeBox>(m_material, 200, 200, 1);
        terrain_body->AddCollisionShape(shape, ChFrame<>(ChVector3d(0, 0, -1), QUNIT));
        terrain_body->EnableCollision(true);

        auto visual_asset = chrono_types::make_shared<ChVisualShapeBox>(200, 200, 1);
        visual_asset->SetColor(ChColor(0.1f, 0.1f, 0.1f));
        terrain_body->AddVisualShape(visual_asset, ChFrame<>(ChVector3d(0, 0, -1)));

        system->Add(terrain_body);
    }
};

int main(int argc, char* argv[]) {
    SetChronoDataPath("/usr/local/share/chrono/data/");
    vehicle::SetDataPath("/usr/local/share/chrono/data/vehicle/");

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

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Chrono ROS2 HMMWV Control");
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

    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<JoySubscriberNode>();

    double step_size = 2e-3;
    while (vis->Run() && rclcpp::ok()) {
        rclcpp::spin_some(ros_node);

        chrono::vehicle::DriverInputs driver_inputs;
        driver_inputs.m_steering = g_steering;
        driver_inputs.m_throttle = g_throttle;
        driver_inputs.m_braking = g_braking;

        my_hmmwv.Synchronize(my_hmmwv.GetSystem()->GetChTime(), driver_inputs, custom_terrain);
        vis->Synchronize(my_hmmwv.GetSystem()->GetChTime(), driver_inputs);

        my_hmmwv.Advance(step_size);
        vis->Advance(step_size);

        vis->BeginScene();
        vis->Render();
        vis->EndScene();
    }

    rclcpp::shutdown();
    return 0;
}
