// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/common/thread/recurrent_thread.hpp>
#include "BaseState.h"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>

class CtrlFSM
{
public:
    CtrlFSM(std::shared_ptr<BaseState> initstate)
    {
        // Initialize FSM states
        states.push_back(std::move(initstate));

    }

    CtrlFSM(YAML::Node cfg)
    {
        auto fsms = cfg["_"]; // enabled FSMs

        // register FSM string map; used for state transition
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        // Initialize FSM states
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end()) {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            auto state_instance = fsm_class->second(id, fsm_name);
            add(state_instance);
        }
        if (cfg["initial"]) initial_state_ = FSMStringMap.right.at(cfg["initial"].as<std::string>());
    }

    void start() 
    {
        // The configured initial state can stand without running the policy.
        currentState = states[0];
        if (initial_state_)
            for (const auto& state : states) if (state->isState(initial_state_)) currentState = state;
        currentState->pre_run();
        currentState->enter();
        currentState->run();
        currentState->post_run();

        spdlog::info("FSM: Start {}", currentState->getStateString());
        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", 0, this->dt * 1e6, &CtrlFSM::run_, this);
    }

    // Called from the main thread, never from the recurrent callback.
    // Stop publishing active commands and join the policy before the final state.
    void stop(int final_state = 0)
    {
        if (!fsm_thread_) return;
        fsm_thread_->Wait();
        fsm_thread_.reset();
        currentState->exit();
        currentState.reset();

        if (final_state == 0) return;
        std::shared_ptr<BaseState> final;
        for (auto& state : states)
            if (state->isState(final_state)) final = state;
        if (!final) throw std::runtime_error("FSM: final state not found");

        // Keep final Passive damping flowing for 100 ms before closing DDS.
        final->pre_run();
        final->enter();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        do {
            final->run();
            final->post_run();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (std::chrono::steady_clock::now() >= deadline) break;
            final->pre_run();
        } while (true);
        final->exit();
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for(auto & s : states)
        {
            if(s->isState(state->getState()))
            {
                spdlog::error("FSM: State_{} already exists", state->getStateString());
                std::exit(0);
            }
        }

        states.push_back(std::move(state));
    }
    
    ~CtrlFSM()
    {
        stop();
        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;
private:
    const double dt = 0.001;

    void run_()
    {
        currentState->pre_run();
        
        // Check if need to change state
        int nextStateMode = 0;
        for (const auto& check : currentState->safety_checks)
            if (check.first()) { nextStateMode = check.second; break; }
        for(int i(0); nextStateMode == 0 && i<currentState->registered_checks.size(); i++)
        {
            if(currentState->registered_checks[i].first() &&
               currentState->can_transition_to(currentState->registered_checks[i].second))
            {
                nextStateMode = currentState->registered_checks[i].second;
                break;
            }
        }

        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    break;
                }
            }
        }

        // pre_run owns the publisher lock through transitions and command writes.
        currentState->run();
        currentState->post_run();
    }

    std::shared_ptr<BaseState> currentState;
    int initial_state_ = 0;
    unitree::common::RecurrentThreadPtr fsm_thread_;
};
