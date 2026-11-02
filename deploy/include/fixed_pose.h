#pragma once

#include "LinearInterpolator.h"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>

// Fixed poses and measured startup angles are all in SDK motor order.
class FixedPose {
public:
    explicit FixedPose(const YAML::Node& cfg)
        : kp(cfg["kp"].as<std::vector<float>>()), kd(cfg["kd"].as<std::vector<float>>()),
          ts_(cfg["ts"].as<std::vector<float>>()), qs_(cfg["qs"].as<std::vector<std::vector<float>>>()) {
        if (kp.size() != 12 || kd.size() != 12 || ts_.size() < 2 || ts_.size() != qs_.size() ||
            ts_.front() != 0 || !qs_.front().empty())
            throw std::runtime_error("Fixed pose requires 12 gains, increasing ts from 0, and qs[0]: []");
        for (std::size_t i = 0; i < ts_.size(); ++i) {
            if (!std::isfinite(ts_[i]) || (i && ts_[i] <= ts_[i - 1]))
                throw std::runtime_error("Fixed pose times must be finite and strictly increasing");
            if (i && qs_[i].size() != 12) throw std::runtime_error("Fixed pose requires 12 joint targets");
            for (float q : qs_[i]) if (!std::isfinite(q)) throw std::runtime_error("Non-finite fixed pose");
        }
        for (auto gains : {kp, kd})
            for (float gain : gains)
                if (!std::isfinite(gain) || gain < 0) throw std::runtime_error("Invalid fixed pose gain");
    }
    void start(const std::vector<float>& measured) {
        if (measured.size() != 12) throw std::runtime_error("Fixed pose needs 12 measured angles");
        for (float q : measured) if (!std::isfinite(q)) throw std::runtime_error("Non-finite measured angle");
        qs_[0] = measured;
    }
    std::vector<float> sample(float elapsed) const { return linear_interpolate(elapsed, ts_, qs_); }
    bool ready(float elapsed) const { return elapsed >= ts_.back(); }
    const std::vector<float> kp, kd;
private:
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
};
