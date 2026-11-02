#pragma once

#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace a2_controls {

// SDK mapping: physical R1 -> RB, physical R2 -> RT.
// Read the SDK's button/axis state without changing its filters or edge detection.
inline bool shutdown_requested(const unitree::common::UnitreeJoystick& joystick)
{
    return joystick.RB.pressed && joystick.RT.pressed;
}

class SpeedControl {
    struct Setting {
        float initial, step, minimum, maximum;
        explicit Setting(const YAML::Node& cfg)
            : initial(cfg["initial"].as<float>()), step(cfg["step"].as<float>()),
              minimum(cfg["min"].as<float>()), maximum(cfg["max"].as<float>()) {
            if (!std::isfinite(initial) || !std::isfinite(step) || !std::isfinite(minimum) ||
                !std::isfinite(maximum) || minimum < 0 || maximum > 1 || minimum > maximum ||
                initial < minimum || initial > maximum || step <= 0)
                throw std::runtime_error("Speed scales require 0 <= min <= initial <= max <= 1 and step > 0");
        }
    };
public:
    explicit SpeedControl(const YAML::Node& cfg)
        : linear_(cfg["linear"]), angular_(cfg["angular"]), scales_{linear_.initial, angular_.initial} {}

    // Raw SDK packet bits only suppress speed changes while the exit chord's
    // filtered RT is still rising. SDK axis/button fields are never modified.
    bool update(const unitree::common::UnitreeJoystick& joy, bool exit_buttons_down) {
        if (exit_buttons_down || shutdown_requested(joy)) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        const auto previous = scales_;
        scales_[0] = std::clamp(scales_[0] + linear_.step *
            (int(joy.RB.on_pressed) - int(joy.LB.on_pressed)), linear_.minimum, linear_.maximum);
        scales_[1] = std::clamp(scales_[1] + angular_.step *
            (int(joy.RT.on_pressed) - int(joy.LT.on_pressed)), angular_.minimum, angular_.maximum);
        return scales_ != previous;
    }

    std::array<float, 2> scales() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return scales_;
    }
private:
    Setting linear_, angular_;
    mutable std::mutex mutex_;
    std::array<float, 2> scales_;
};

inline float velocity_command(float axis, float minimum, float maximum, float scale) {
    axis = std::clamp(axis, -1.0f, 1.0f);
    return axis * (axis >= 0 ? maximum : -minimum) * scale;
}

} // namespace a2_controls
