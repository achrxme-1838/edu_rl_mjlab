// Exercise thread shutdown and final-state publishing without DDS channels.
#include "FSM/CtrlFSM.h"
#include <atomic>
#include <filesystem>
#include <iostream>

class TestState : public BaseState {
public:
    TestState(int id, const std::string& name) : BaseState(id, name) {}
    void enter() override {
        ++enters;
        if (isState(3)) {
            policy_running = true;
            policy = std::thread([this] {
                while (policy_running) {
                    ++policy_steps;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            });
        }
    }
    void pre_run() override { ++prepares; }
    void run() override { ++runs; }
    void post_run() override { ++publishes; }
    bool can_transition_to(int next) override { return next != 3 || ready_for_policy; }
    void exit() override {
        policy_running = false;
        if (policy.joinable()) policy.join();
        ++exits;
    }
    std::atomic<int> enters{0}, runs{0}, publishes{0}, prepares{0}, exits{0}, policy_steps{0};
    std::atomic<bool> policy_running{false};
    std::atomic<bool> ready_for_policy{true};
    std::thread policy;
};

int main(int argc, char** argv) {
    auto check = [](bool value) { if (!value) throw std::runtime_error("FSM shutdown mismatch"); };
    auto wait_for = [&](auto condition) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!condition() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(condition());
    };
    check(argc == 2);
    // Use the real YAML and FSM scheduler with states that never open DDS.
    for (const auto type : {"State_Passive", "State_FixStand", "State_RLBase"})
        getFsmMap()[type] = [](int id, std::string name) { return std::make_shared<TestState>(id, name); };
    const auto cfg = YAML::LoadFile((std::filesystem::path(argv[1]) /
        "deploy/robots/a2/config/config.yaml").string())["FSM"];
    {
        CtrlFSM fsm(cfg);
        auto state = [&](int id) {
            for (const auto& item : fsm.states)
                if (item->isState(id)) return std::dynamic_pointer_cast<TestState>(item);
            throw std::runtime_error("Missing test state");
        };
        auto passive = state(1), stand = state(2), velocity = state(3), low = state(4);
        stand->ready_for_policy = false;
        stand->registered_checks.emplace_back([] { return true; }, 3);
        std::atomic<bool> fault{false};
        velocity->safety_checks.emplace_back([&] { return fault.load(); }, 1);
        velocity->registered_checks.emplace_back([&] { return fault.load(); }, 4);
        fsm.start();
        wait_for([&] { return stand->runs >= 5; });
        check(stand->enters == 1 && passive->enters == 0 && velocity->enters == 0);
        stand->ready_for_policy = true;
        wait_for([&] { return velocity->policy_steps > 0; });
        fault = true;
        wait_for([&] { return passive->runs > 0; });
        check(low->enters == 0 && !velocity->policy_running);
        fsm.stop(1);
        check(!velocity->policy.joinable());
    }
    for (int initial_id : {1, 2, 3, 4}) {
        auto passive = std::make_shared<TestState>(1, "Passive");
        auto stand = std::make_shared<TestState>(2, "FixStand");
        auto velocity = std::make_shared<TestState>(3, "Velocity");
        auto low = std::make_shared<TestState>(4, "LowStand");
        std::vector<std::shared_ptr<TestState>> states{passive, stand, velocity, low};
        auto active = states[initial_id - 1];
        CtrlFSM fsm(active);
        for (const auto& state : states) if (state != active) fsm.add(state);
        // Holding a normal transition button cannot leave Passive during stop().
        passive->registered_checks.emplace_back([] { return true; }, initial_id);
        fsm.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const auto start = std::chrono::steady_clock::now();
        fsm.stop(1);
        check(std::chrono::steady_clock::now() - start >= std::chrono::milliseconds(100));
        check(passive->publishes > 0 && passive->exits > 0);
        check(active->exits > 0 && !velocity->policy_running && !velocity->policy.joinable());
        for (const auto& state : states) {
            check(state->prepares == state->publishes);
            check(state->runs == state->publishes);
        }
        const int last_published = passive->publishes, last_policy_step = velocity->policy_steps;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        check(passive->publishes == last_published && velocity->policy_steps == last_policy_step);
        fsm.stop(); // Repeated cleanup must be harmless.
    }
    std::cout << "PASS: configured startup, policy entry gate, safety priority, shutdown from all 4 states, 100 ms Passive and joined threads\n";
}
