#include "unitree_joystick_dsl.hpp"
#include "a2_controls.h"
#include "fixed_pose.h"
#include <cmath>
#include <filesystem>
#include <iostream>

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
    std::cout << "PASS: Start/A, speed steps/limits, SDK defaults, shutdown priority and fixed-pose interpolation\n";
}
