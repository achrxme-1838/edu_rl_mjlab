// Offline tests: use real HG packets, but never create DDS channels.
#include "JointTest.h"
#include <iostream>
#include <limits>

using namespace a2_joint_test;

void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void near(float actual, float expected) {
    check(std::abs(actual - expected) < 1e-6f, "Joint angle or gain mismatch");
}

template <typename F> void rejects(F action) {
    bool rejected = false;
    try { action(); } catch (const std::exception&) { rejected = true; }
    check(rejected, "Invalid input was accepted");
}

int main(int argc, char** argv) {
    try {
        check(argc == 2, "Supply repository root");
        const std::filesystem::path root(argv[1]);
        const auto cfg = Config::load(root / "deploy/robots/a2/config/deploy.yaml", root / "deploy/robots/a2/config/config.yaml");
        const std::array<int, 12> expected_ids{3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8};
        const auto source = YAML::LoadFile((root / "networks/01_EDU_A2_STAIR/source_model.json").string());
        for (int i = 0; i < 12; ++i) {
            check(joint_name(i) == source["joint_names"][i].as<std::string>(), "Policy joint name mismatch");
            check(cfg.motor_ids[i] == expected_ids[i], "SDK motor ID mismatch");
            near(cfg.default_sdk[cfg.motor_ids[i]], source["default_joint_pos"][i].as<float>());
        }

        auto initial = cfg.default_sdk;
        for (float& q : initial) q += 0.1f;
        Sequence sequence(cfg, initial, 0);
        check(sequence.sample(0).q == initial, "Must start from measured pose");
        near(sequence.sample(0).gain, 0);
        const auto halfway = sequence.sample(2.5);
        for (int i = 0; i < 12; ++i) near(halfway.q[i], (initial[i] + cfg.default_sdk[i]) / 2);
        check(!sequence.next_joint(2.5), "Must ignore Space during initial pose transition");
        double now = 5;
        sequence.sample(now);
        for (int i = 0; i < 12; ++i) {
            check(sequence.policy_id() == i, "EDU order mismatch");
            check(sequence.description().find(joint_label(i)) != std::string::npos, "Missing expected joint name");
            now += 0.75;
            const auto target = sequence.sample(now);
            for (int motor = 0; motor < 12; ++motor)
                near(target.q[motor], cfg.default_sdk[motor] + (motor == cfg.motor_ids[i] ? 0.05f : 0.0f));
            check(sequence.next_joint(now), "Space did not advance");
            check(sequence.sample(now).q == target.q, "Space introduced a target jump");
            check(!sequence.next_joint(now + 0.2), "Must ignore Space during return");
            now += 1;
            check(sequence.sample(now).q == cfg.default_sdk, "Joint did not return to default pose");
        }
        check(sequence.phase() == Sequence::Phase::Done, "Sequence did not finish");
        check(sequence.sample(now + 20).q == cfg.default_sdk, "Final pose is not held");

        Motion sdk_motion;
        sdk_motion.sdk_order = true;
        Sequence sdk(cfg, initial, 0, sdk_motion);
        for (int i = 0; i < 12; ++i) check(cfg.motor_ids[sdk.order()[i]] == i, "SDK order mismatch");
        check(joint_label(sdk.policy_id()) == "right front hip", "SDK first joint label mismatch");

        LowCmd command{};
        fill_command(command, cfg, Target{cfg.default_sdk});
        check(command.mode_pr() == 0 && command.mode_machine() == 1, "HG mode mismatch");
        check(command.crc() == command_crc(command), "CRC mismatch");
        // Reference packet produced by the SDK's Python CRC before migration, without DDS.
        check(command.crc() == 0xdca2891d, "Default packet differs from the original SDK packet");
        auto target = Target{cfg.default_sdk, 0.5f};
        target.q[3] += 0.05f;
        fill_command(command, cfg, target);
        for (int i = 0; i < 12; ++i) {
            const auto& motor = command.motor_cmd()[i];
            check(motor.mode() == 1, "Active motor disabled");
            near(motor.q(), target.q[i]);
            near(motor.kp(), cfg.kp[i] * 0.5f);
            near(motor.kd(), cfg.kd[i]);
            near(motor.dq(), 0);
            near(motor.tau(), 0);
        }
        fill_command(command, cfg, target, true);
        for (int i = 0; i < 12; ++i) {
            near(command.motor_cmd()[i].kp(), 0);
            near(command.motor_cmd()[i].kd(), cfg.passive_kd[i]);
        }
        for (std::size_t i = 12; i < command.motor_cmd().size(); ++i)
            check(command.motor_cmd()[i] == unitree_hg::msg::dds_::MotorCmd_{}, "Unused motor slot changed");

        const SensorSample valid{cfg.default_sdk, 1, 5};
        check_sample(valid, 5.01);
        rejects([&] { check_sample(std::nullopt, 5); });
        rejects([&] { check_sample(valid, 5.21); });
        auto wrong_robot = valid;
        wrong_robot.mode_machine = 2;
        rejects([&] { check_sample(wrong_robot, 5); });
        auto nan_state = valid;
        nan_state.q[0] = std::numeric_limits<float>::quiet_NaN();
        rejects([&] { check_sample(nan_state, 5); });
        rejects([&] { fill_command(command, cfg, Target{nan_state.q}); });
        rejects([&] { fill_command(command, cfg, Target{cfg.default_sdk, 2}); });
        rejects([&] { Motion{0.16, 3, 5}.validate(); });
        rejects([&] { Motion{0.05, 1, 5}.validate(); });
        rejects([&] { Motion{0.05, 3, 2}.validate(); });
        rejects([&] { Motion{std::numeric_limits<double>::quiet_NaN(), 3, 5}.validate(); });
        std::cout << "PASS: mapping, pose, all 12 isolated joints, Space return, HG packet/CRC, Passive and state guards\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
