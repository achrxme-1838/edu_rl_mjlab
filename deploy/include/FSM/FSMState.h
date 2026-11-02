#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"
#include "a2_controls.h"
#include <atomic>

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        // A latched shutdown takes precedence over every normal button transition.
        safety_checks.emplace_back(
            [] { return shutdown_requested.load(); }, FSMStringMap.right.at("Passive"));

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

        // register for all states
        safety_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
        if (a2_controls::shutdown_requested(lowstate->joystick))
            shutdown_requested.store(true);
        if (speed_control && !shutdown_requested.load()) {
            unitree::common::REMOTE_DATA_RX remote{};
            {
                std::lock_guard<std::mutex> lock(lowstate->mutex_);
                std::memcpy(&remote, lowstate->msg_.wireless_remote().data(), sizeof(remote));
            }
            const bool exit_buttons = remote.RF_RX.btn.components.R1 && remote.RF_RX.btn.components.R2;
            if (speed_control->update(lowstate->joystick, exit_buttons)) {
                const auto scales = speed_control->scales();
                spdlog::info("Speed scale: linear {:.1f}, angular {:.1f}", scales[0], scales[1]);
            }
        }
        if(keyboard) keyboard->update();
        lowcmd->lock();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
    inline static std::atomic<bool> shutdown_requested{false};
    inline static std::unique_ptr<a2_controls::SpeedControl> speed_control;
};
