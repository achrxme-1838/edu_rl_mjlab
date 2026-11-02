#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>
#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <yaml-cpp/yaml.h>

namespace a2_joint_test {

using Joints = std::array<float, 12>;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
constexpr double kStateTimeout = 0.2;

inline std::string joint_name(int policy_id) {
    const std::array<std::string, 4> legs{"FL", "FR", "RL", "RR"};
    const std::array<std::string, 3> joints{"hip", "thigh", "calf"};
    return legs.at(policy_id % 4) + "_" + joints.at(policy_id / 4) + "_joint";
}

inline std::string joint_label(int policy_id) {
    const std::array<std::string, 4> legs{"left front", "right front", "left rear", "right rear"};
    const std::array<std::string, 3> joints{"hip", "thigh", "calf"};
    return legs.at(policy_id % 4) + " " + joints.at(policy_id / 4);
}

inline bool finite(const Joints& values) {
    return std::all_of(values.begin(), values.end(), [](float x) { return std::isfinite(x); });
}

struct Config {
    std::array<int, 12> motor_ids{};
    Joints default_sdk{}, kp{}, kd{}, passive_kd{};

    static Config load(const std::filesystem::path& policy, const std::filesystem::path& fsm) {
        const auto cfg = YAML::LoadFile(policy.string());
        const auto fsm_cfg = YAML::LoadFile(fsm.string());
        auto read_joints = [](const YAML::Node& node) {
            if (!node.IsSequence() || node.size() != 12)
                throw std::runtime_error("Pose and gains must contain 12 finite values");
            Joints values{};
            for (int i = 0; i < 12; ++i) values[i] = node[i].as<float>();
            if (!finite(values)) throw std::runtime_error("Pose and gains must be finite");
            return values;
        };
        Config result;
        const auto ids = cfg["joint_ids_map"].as<std::vector<int>>();
        auto sorted_ids = ids;
        std::sort(sorted_ids.begin(), sorted_ids.end());
        const std::vector<int> expected{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        if (sorted_ids != expected)
            throw std::runtime_error("joint_ids_map must be a permutation of motor IDs 0..11");
        std::copy(ids.begin(), ids.end(), result.motor_ids.begin());
        const auto pose = read_joints(cfg["default_joint_pos"]);
        for (int i = 0; i < 12; ++i) result.default_sdk[ids[i]] = pose[i];
        result.kp = read_joints(cfg["stiffness"]);
        result.kd = read_joints(cfg["damping"]);
        result.passive_kd = read_joints(fsm_cfg["FSM"]["Passive"]["kd"]);
        for (const auto& gains : {result.kp, result.kd, result.passive_kd})
            for (float gain : gains)
                if (gain < 0) throw std::runtime_error("PD gains cannot be negative");
        return result;
    }
};

struct Motion {
    double amplitude = 0.05;
    double period = 3.0;
    double stand_duration = 5.0;
    bool sdk_order = false;

    void validate() const {
        if (!std::isfinite(amplitude) || !std::isfinite(period) || !std::isfinite(stand_duration) ||
            amplitude <= 0 || amplitude > 0.15 || period < 2 || stand_duration < 3)
            throw std::runtime_error("Require 0 < amplitude <= 0.15, period >= 2, stand-duration >= 3");
    }
};

inline double smoothstep(double value) {
    value = std::clamp(value, 0.0, 1.0);
    return value * value * (3.0 - 2.0 * value);
}

struct Target {
    Joints q;
    float gain = 1;
};

class Sequence {
public:
    enum class Phase { Stand, Wave, Return, Done };

    Sequence(const Config& cfg, Joints initial, double now, Motion motion = {})
        : cfg_(cfg), initial_(initial), motion_(motion), phase_start_(now) {
        motion_.validate();
        if (!finite(initial)) throw std::runtime_error("Invalid initial joint positions");
        std::iota(order_.begin(), order_.end(), 0);
        if (motion.sdk_order)
            std::sort(order_.begin(), order_.end(), [&](int a, int b) { return cfg.motor_ids[a] < cfg.motor_ids[b]; });
    }

    Target sample(double now) {
        double elapsed = std::max(0.0, now - phase_start_);
        if (phase_ == Phase::Stand && elapsed >= motion_.stand_duration) {
            phase_ = Phase::Wave;
            phase_start_ = now;
            elapsed = 0;
        } else if (phase_ == Phase::Return && elapsed >= 1.0) {
            ++index_;
            phase_ = index_ < 12 ? Phase::Wave : Phase::Done;
            phase_start_ = now;
            elapsed = 0;
        }
        Target target{cfg_.default_sdk};
        if (phase_ == Phase::Stand || phase_ == Phase::Return) {
            const auto& start = phase_ == Phase::Stand ? initial_ : return_start_;
            const double duration = phase_ == Phase::Stand ? motion_.stand_duration : 1.0;
            const double blend = smoothstep(elapsed / duration);
            for (int i = 0; i < 12; ++i) target.q[i] = start[i] + blend * (target.q[i] - start[i]);
            if (phase_ == Phase::Stand) target.gain = smoothstep(elapsed);
        } else if (phase_ == Phase::Wave) {
            constexpr double pi = 3.14159265358979323846;
            target.q[cfg_.motor_ids[policy_id()]] +=
                motion_.amplitude * smoothstep(elapsed / 0.5) * std::sin(2 * pi * elapsed / motion_.period);
        }
        return target;
    }

    bool next_joint(double now) {
        if (phase_ != Phase::Wave) return false;
        return_start_ = sample(now).q;
        phase_ = Phase::Return;
        phase_start_ = now;
        return true;
    }

    Phase phase() const { return phase_; }
    int index() const { return index_; }
    int policy_id() const { return index_ < 12 ? order_[index_] : -1; }
    const std::array<int, 12>& order() const { return order_; }

    std::string description() const {
        if (phase_ == Phase::Stand)
            return "Moving to EDU default pose over " + std::to_string(motion_.stand_duration) + "s...";
        if (phase_ == Phase::Done)
            return "All 12 joints completed. Holding default pose. Q / Esc / Ctrl-C: passive stop.";
        const std::string prefix = phase_ == Phase::Wave ? "TEST" : "RETURN";
        return "[" + prefix + " " + std::to_string(index_ + 1) + "/12] expected: " + joint_label(policy_id()) +
            " (" + joint_name(policy_id()) + ") | EDU joint " + std::to_string(policy_id()) +
            " -> motor ID " + std::to_string(cfg_.motor_ids[policy_id()]);
    }

private:
    const Config& cfg_;
    Joints initial_, return_start_{};
    Motion motion_;
    std::array<int, 12> order_{};
    Phase phase_ = Phase::Stand;
    double phase_start_;
    int index_ = 0;
};

struct SensorSample {
    Joints q;
    int mode_machine;
    double received_at;
};

inline void check_sample(const std::optional<SensorSample>& sample, double now) {
    if (!sample || now - sample->received_at > kStateTimeout)
        throw std::runtime_error("LowState is missing or stale (>0.2s); stopping");
    if (sample->mode_machine != 1)
        throw std::runtime_error("Expected A2 mode_machine=1, got " + std::to_string(sample->mode_machine));
    if (!finite(sample->q)) throw std::runtime_error("Invalid measured joint positions; stopping");
}

inline uint32_t command_crc(const LowCmd& command) {
    // Same packet layout and CRC as the deployment SDK publisher; copy to avoid aliasing.
    static_assert(std::is_trivially_copyable_v<LowCmd> && sizeof(LowCmd) % 4 == 0);
    std::array<uint32_t, sizeof(LowCmd) / 4> words{};
    std::memcpy(words.data(), &command, sizeof(command));
    return crc32_core(words.data(), words.size() - 1);
}

inline void fill_command(LowCmd& command, const Config& cfg, const Target& target, bool passive = false) {
    if (!finite(target.q) || !std::isfinite(target.gain) || target.gain < 0 || target.gain > 1)
        throw std::runtime_error("Invalid joint targets or gain");
    command.mode_pr() = 0;
    command.mode_machine() = 1;
    for (int i = 0; i < 12; ++i) {
        auto& motor = command.motor_cmd()[i];
        motor.mode() = 1;
        motor.q() = target.q[i];
        motor.dq() = motor.tau() = 0;
        motor.kp() = passive ? 0 : cfg.kp[i] * target.gain;
        motor.kd() = passive ? cfg.passive_kd[i] : cfg.kd[i];
    }
    command.crc() = command_crc(command);
}

} // namespace a2_joint_test
