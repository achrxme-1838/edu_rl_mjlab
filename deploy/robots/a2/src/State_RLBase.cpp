#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto policy_dir = param::parser_policy_dir(param::deploy_config["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        param::deploy_config,
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(
            FSMState::lowstate, [] { return FSMState::speed_control->scales(); })
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "policy.onnx");

    this->safety_checks.emplace_back(
        std::make_pair(
            [&]()->bool{
                return policy_failed || invalid_orientation;
            },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    std::lock_guard<std::mutex> lock(target_mutex);
    unitree::write_joint_position_targets(lowcmd->msg_, env->robot->data.joint_ids_map,
                                         position_targets);
}
