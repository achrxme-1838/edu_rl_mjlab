// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
#pragma once

#include "FSMState.h"
#include "fixed_pose.h"
#include <chrono>

// Reused by FixStand and LowStand; each state reads its own pose and gains.
class State_FixStand : public FSMState
{
public:
    State_FixStand(int state, std::string state_string = "FixStand")
        : FSMState(state, state_string), pose_(param::config["FSM"][state_string]) {}

    void enter() override
    {
        std::vector<float> measured;
        {
            std::lock_guard<std::mutex> lock(lowstate->mutex_);
            for (std::size_t i = 0; i < pose_.kp.size(); ++i)
                measured.push_back(lowstate->msg_.motor_state()[i].q());
        }
        // Startup and every mode change start at the measured pose, not zero commands.
        pose_.start(measured);
        for (std::size_t i = 0; i < pose_.kp.size(); ++i) {
            auto& motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = pose_.kp[i];
            motor.kd() = pose_.kd[i];
            motor.dq() = motor.tau() = 0;
        }
        started_ = std::chrono::steady_clock::now();
    }

    void run() override
    {
        const auto q = pose_.sample(elapsed());
        for (std::size_t i = 0; i < q.size(); ++i)
            lowcmd->msg_.motor_cmd()[i].q() = q[i];
    }

    bool can_transition_to(int next) override
    {
        // A/shutdown may interrupt a pose change; policy starts only after standing.
        return FSMStringMap.left.at(next) != "Velocity" || pose_.ready(elapsed());
    }

private:
    float elapsed() const {
        return std::chrono::duration<float>(std::chrono::steady_clock::now() - started_).count();
    }
    FixedPose pose_;
    std::chrono::steady_clock::time_point started_;
};

REGISTER_FSM(State_FixStand)
