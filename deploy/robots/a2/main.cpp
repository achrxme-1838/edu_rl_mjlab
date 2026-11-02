#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "network_config.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    // Validate the copied model before opening DDS channels or creating publishers.
    try {
        param::load_deploy_config();
        FSMState::speed_control = std::make_unique<a2_controls::SpeedControl>(
            param::deploy_config["commands"]["base_velocity"]["speed_control"]);
        // Validate both fixed poses before opening DDS or sending motor commands.
        FixedPose normal_pose(param::config["FSM"]["FixStand"]);
        FixedPose low_pose(param::config["FSM"]["LowStand"]);
        const auto model = policy_bundle::model_path(
            param::repo_dir, param::deploy_config["policy_dir"].as<std::string>());
        isaaclab::OrtRunner probe(model.string());
        const auto actions = probe.act({{"obs", std::vector<float>(225, 0.0f)}});
        if (actions.size() != 12 || !std::all_of(actions.begin(), actions.end(), [](float x) { return std::isfinite(x); }))
            throw std::runtime_error("A2 policy must produce 12 finite actions");
        spdlog::info("Policy ready: {}", model.string());
        if (vm.count("check-policy")) {
            std::cout << "Policy check complete (no DDS channels opened).\n";
            return 0;
        }
    } catch (const std::exception& error) {
        spdlog::error("Policy check failed before DDS: {}", error.what());
        return 1;
    }

    std::string network;
    try {
        const auto cli_network = vm.count("network")
            ? std::make_optional(vm["network"].as<std::string>()) : std::nullopt;
        network = a2_network::resolve(param::config, cli_network);
    } catch (const std::exception& error) {
        spdlog::error("Network configuration failed before DDS: {}", error.what());
        return 1;
    }

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     A2 Controller \n";

    // Unitree DDS Config
    spdlog::info("DDS network interface: {}", network);
    unitree::robot::ChannelFactory::Instance()->Init(0, network);

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 1;
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Automatically standing in FixStand (no policy).\n";
    std::cout << "Press [Start] to toggle FixStand / policy after standing; [A] for LowStand.\n";
    std::cout << "From LowStand, [Start] returns to FixStand first.\n";
    std::cout << "Speed: [R1]/[L1] linear +/-; [R2]/[L2] angular +/-.\n";
    std::cout << "Press [R1 + R2] together to stop the controller (Passive damping, then exit).\n";

    while (!FSMState::shutdown_requested.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    spdlog::info("R1 + R2: stopping controller with Passive damping.");
    fsm->stop(FSMStringMap.right.at("Passive"));
    fsm.reset();
    FSMState::lowcmd.reset();
    FSMState::lowstate.reset();
    spdlog::info("Controller stopped.");
    return 0;
}

