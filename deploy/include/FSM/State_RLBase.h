// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include <atomic>
#include <mutex>

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state_mode, std::string state_string);
    
    void enter()
    {
        // Finish reset before the 1 kHz FSM can publish the first position target.
        env->reset();
        {
            std::lock_guard<std::mutex> lock(target_mutex);
            position_targets = env->action_manager->processed_actions();
        }
        invalid_orientation = isaaclab::mdp::bad_orientation(env.get(), 1.0f);
        policy_failed = false;
        // set gain
        for (int i = 0; i < env->robot->data.joint_stiffness.size(); ++i)
        {
            lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
            lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
            lowcmd->msg_.motor_cmd()[i].dq() = 0;
            lowcmd->msg_.motor_cmd()[i].tau() = 0;
        }

        // Start policy thread
        policy_thread_running = true;
        policy_thread = std::thread([this]{
            using clock = std::chrono::high_resolution_clock;
            const std::chrono::duration<double> desiredDuration(env->step_dt);
            const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

            // Initialize timing
            auto sleepTill = clock::now() + dt;
            while (policy_thread_running)
            {
                try {
                    env->step();
                    invalid_orientation = isaaclab::mdp::bad_orientation(env.get(), 1.0f);
                    // Keep the 1 kHz publisher independent of ONNX inference time.
                    std::lock_guard<std::mutex> lock(target_mutex);
                    position_targets = env->action_manager->processed_actions();
                } catch (const std::exception& e) {
                    spdlog::error("Policy execution failed: {}", e.what());
                    policy_failed = true;
                    break;
                }

                // Sleep
                std::this_thread::sleep_until(sleepTill);
                sleepTill += dt;
            }
        });
    }

    void run();
    
    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }
    }

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

    std::thread policy_thread;
    std::atomic<bool> policy_thread_running{false};
    std::atomic<bool> policy_failed{false};
    std::atomic<bool> invalid_orientation{false};
    std::mutex target_mutex;
    std::vector<float> position_targets;
};

REGISTER_FSM(State_RLBase)
