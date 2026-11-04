#include "unitree_joystick_dsl.hpp"
#include "a2_controls.h"
#include "fixed_pose.h"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <unitree/idl/hg/LowState_.hpp>
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

struct JoystickSensorFrame {
    std::mutex mutex_;
    unitree_hg::msg::dds_::LowState_ msg_;
    unitree::common::UnitreeJoystick joystick;
};

int main(int argc, char** argv) {
    using namespace unitree::common;
    auto check = [](bool value) { if (!value) throw std::runtime_error("Joystick/control mismatch"); };
    auto near = [&](float a, float b) { check(std::fabs(a - b) < 1e-5f); };
    check(argc == 2);
    const auto root = std::filesystem::path(argv[1]);
    const auto fsm = YAML::LoadFile((root / "deploy/robots/a2/config/config.yaml").string())["FSM"];
    const auto speed_cfg = YAML::LoadFile((root / "deploy/robots/a2/config/deploy.yaml").string())
        ["commands"]["base_velocity"]["speed_control"];
    auto transition = [&](const std::string& state, const UnitreeJoystick& joy) {
        for (auto it : fsm[state]["transitions"]) {
            dsl::Parser parser(it.second.as<std::string>());
            if (dsl::Compile(*parser.Parse())(joy)) return it.first.as<std::string>();
        }
        return state;
    };
    check(fsm["initial"].as<std::string>() == "FixStand");
    UnitreeJoystick joy;
    REMOTE_DATA_RX key{};
    joy.extract(key);
    key.RF_RX.btn.components.Start = 1;
    joy.extract(key);
    check(transition("FixStand", joy) == "Velocity");
    check(transition("Velocity", joy) == "FixStand");
    check(transition("LowStand", joy) == "FixStand");
    joy.extract(key);
    check(transition("FixStand", joy) == "FixStand"); // No repeated toggle while held.
    key.RF_RX.btn.components.Start = 0;
    joy.extract(key);
    key.RF_RX.btn.components.A = key.RF_RX.btn.components.Start = 1;
    joy.extract(key);
    check(transition("Velocity", joy) == "LowStand"); // A wins over Start.
    check(transition("FixStand", joy) == "LowStand");
    check(transition("LowStand", joy) == "LowStand");

    a2_controls::SpeedControl speed(speed_cfg);
    UnitreeJoystick speed_joy;
    REMOTE_DATA_RX buttons{};
    auto tick = [&](int count = 1) {
        for (int i = 0; i < count; ++i) {
            speed_joy.extract(buttons);
            speed.update(speed_joy, buttons.RF_RX.btn.components.R1 && buttons.RF_RX.btn.components.R2);
        }
    };
    buttons.RF_RX.btn.components.R1 = 1; tick(64);
    near(speed.scales()[0], 0.6f); near(speed.scales()[1], 0.5f);
    buttons.RF_RX.btn.value = 0; tick(64);
    buttons.RF_RX.btn.components.L1 = 1; tick(64);
    near(speed.scales()[0], 0.5f);
    buttons.RF_RX.btn.value = 0; tick(64);
    buttons.RF_RX.btn.components.R2 = 1; tick(64);
    near(speed.scales()[1], 0.6f); near(speed.scales()[0], 0.5f);
    buttons.RF_RX.btn.value = 0; tick(64);
    buttons.RF_RX.btn.components.L2 = 1; tick(64);
    near(speed.scales()[1], 0.5f); near(speed.scales()[0], 0.5f);
    for (bool increase : {true, false}) {
        for (int i = 0; i < 20; ++i) {
            buttons.RF_RX.btn.value = 0; tick(64);
            buttons.RF_RX.btn.components.R1 = increase;
            buttons.RF_RX.btn.components.L1 = !increase;
            tick(64);
        }
        near(speed.scales()[0], increase ? 1.0f : 0.0f);
    }
    near(a2_controls::velocity_command(1, -.6, .6, .5), .3);
    near(a2_controls::velocity_command(-1, -.4, .4, .5), -.2);
    near(a2_controls::velocity_command(5, -1, 1, 1), 1);
    near(a2_controls::velocity_command(1, -.6, .6, 0), 0);

    // Exercise the production observation function with SDK-filtered packets.
    const auto deploy_cfg = YAML::LoadFile((root / "deploy/robots/a2/config/deploy.yaml").string());
    auto check_sticks = [&](float ly, float lx, float rx, float ry,
                            const std::array<float, 3>& full_command) {
        auto input = std::make_shared<JoystickSensorFrame>();
        input->msg_.imu_state().quaternion()[0] = 1;
        REMOTE_DATA_RX remote{};
        remote.RF_RX.ly = ly; remote.RF_RX.lx = lx;
        remote.RF_RX.rx = rx; remote.RF_RX.ry = ry;
        input->joystick.extract(remote);
        auto articulation = std::make_shared<unitree::BaseArticulation<std::shared_ptr<JoystickSensorFrame>>>(input);
        isaaclab::ManagerBasedRLEnv env(deploy_cfg, articulation);
        for (const auto scales : {std::array<float, 2>{.5f, .25f},
                                  std::array<float, 2>{1, 1},
                                  std::array<float, 2>{0, 0}}) {
            articulation->data.velocity_scales = scales;
            const auto command = isaaclab::mdp::velocity_commands(&env, YAML::Node{});
            check(command.size() == 3);
            for (int i = 0; i < 3; ++i)
                near(command[i], full_command[i] * .03f * scales[i == 2 ? 1 : 0]);
        }
    };
    check_sticks(0, 0, 0, 0, {0, 0, 0});
    check_sticks(1, 0, 0, 0, {.6f, 0, 0});
    check_sticks(-1, 0, 0, 0, {-.6f, 0, 0});
    check_sticks(0, -1, 0, 0, {0, 0, 1});
    check_sticks(0, 1, 0, 0, {0, 0, -1});
    check_sticks(0, 0, -1, 0, {0, .4f, 0});
    check_sticks(0, 0, 1, 0, {0, -.4f, 0});
    check_sticks(0, 0, 0, 1, {0, 0, 0});
    check_sticks(0, 0, 0, -1, {0, 0, 0});
    check_sticks(1, -1, 1, 1, {.6f, -.4f, 1});
    check_sticks(-1, 1, -1, -1, {-.6f, .4f, -1});
    check_sticks(.005f, -.005f, .005f, 1, {0, 0, 0});

    // Both shoulder buttons must not speed up while RT's SDK filter is rising.
    a2_controls::SpeedControl chord_speed(speed_cfg);
    UnitreeJoystick chord_joy;
    REMOTE_DATA_RX chord{};
    chord.RF_RX.btn.components.R1 = chord.RF_RX.btn.components.R2 = 1;
    for (int i = 0; i < 64; ++i) {
        chord_joy.extract(chord);
        check(!chord_speed.update(chord_joy, true));
    }
    check(a2_controls::shutdown_requested(chord_joy));
    near(chord_speed.scales()[0], .5); near(chord_speed.scales()[1], .5);

    // Keep SDK filtering and trigger thresholds intact.
    UnitreeJoystick defaults;
    REMOTE_DATA_RX axes{};
    axes.RF_RX.ly = .005f; axes.RF_RX.rx = 1;
    axes.RF_RX.btn.components.R2 = 1;
    defaults.extract(axes);
    near(defaults.ly(), 0); near(defaults.rx(), .03f);
    check(!defaults.RT.pressed);
    for (int i = 0; i < 64; ++i) defaults.extract(axes);
    check(defaults.RT.pressed);

    // Snapshot the SDK's filtered input without filtering it again or aliasing
    // the live sensor. This also covers SDK versions without copy assignment.
    auto sensor = std::make_shared<JoystickSensorFrame>();
    sensor->msg_.imu_state().quaternion()[0] = 1;
    unitree::BaseArticulation<std::shared_ptr<JoystickSensorFrame>> robot(sensor);
    check(robot.data.joystick != nullptr);
    REMOTE_DATA_RX packet{};
    for (int frame = 0; frame < 128; ++frame) {
        const bool pressed = frame < 64;
        packet.RF_RX.btn.components.Start = pressed;
        packet.RF_RX.btn.components.R2 = pressed;
        packet.RF_RX.rx = pressed ? 1.0f : 0.0f;
        const float previous_axis = robot.data.joystick->rx();
        sensor->joystick.extract(packet);
        near(robot.data.joystick->rx(), previous_axis);
        for (int read = 0; read < 2; ++read) {
            robot.update();
            const auto* snapshot = robot.data.joystick;
            check(snapshot != &sensor->joystick);
            check(snapshot->start.pressed == pressed);
            check(snapshot->start.on_pressed == (frame == 0));
            check(snapshot->start.on_released == (frame == 64));
            check(snapshot->RT.pressed == sensor->joystick.RT.pressed);
            near(robot.data.joystick->rx(), sensor->joystick.rx());
        }
    }

    for (const auto name : {"FixStand", "LowStand"}) {
        FixedPose pose(fsm[name]);
        std::vector<float> measured(12, .25f);
        pose.start(measured);
        check(pose.sample(0) == measured && !pose.ready(0));
        const float duration = fsm[name]["ts"].as<std::vector<float>>().back();
        check(pose.ready(duration));
        check(pose.sample(duration) == fsm[name]["qs"].as<std::vector<std::vector<float>>>().back());
        measured.assign(12, -.3f);
        pose.start(measured);
        check(pose.sample(0) == measured); // Re-entry uses newly measured angles.
    }
    std::cout << "PASS: Start/A, stick velocity mapping, speed steps/limits, SDK defaults and snapshots, shutdown priority and fixed-pose interpolation\n";
}
