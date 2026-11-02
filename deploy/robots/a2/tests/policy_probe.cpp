// Offline probe: uses the deployment managers and DDS message types without channels.
#include <fstream>
#include <memory>
#include <mutex>
#include <limits>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include "unitree_articulation.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"

struct SensorFrame {
    std::mutex mutex_;
    unitree_hg::msg::dds_::LowState_ msg_;
    unitree::common::UnitreeJoystick joystick;
};

class RecordingPolicy : public isaaclab::Algorithms {
public:
    explicit RecordingPolicy(const std::string& path) : runner(path) {}
    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) override {
        last_obs = obs.at("obs");
        return runner.act(std::move(obs));
    }
    isaaclab::OrtRunner runner;
    std::vector<float> last_obs;
};

void load_frame(SensorFrame& sensor, const YAML::Node& frame) {
    auto q = frame["q_sdk"].as<std::vector<float>>();
    auto dq = frame["dq_sdk"].as<std::vector<float>>();
    for (int i = 0; i < 12; ++i) {
        sensor.msg_.motor_state()[i].q() = q.at(i);
        sensor.msg_.motor_state()[i].dq() = dq.at(i);
    }
    auto quat = frame["quat_wxyz"].as<std::vector<float>>();
    auto gyro = frame["gyro"].as<std::vector<float>>();
    for (int i = 0; i < 4; ++i) sensor.msg_.imu_state().quaternion()[i] = quat.at(i);
    for (int i = 0; i < 3; ++i) sensor.msg_.imu_state().gyroscope()[i] = gyro.at(i);
    auto command = frame["command"].as<std::vector<float>>();
    unitree::common::REMOTE_DATA_RX remote{};
    remote.RF_RX.ly = command.at(0);
    remote.RF_RX.lx = -command.at(1);
    remote.RF_RX.rx = -command.at(2);
    sensor.joystick.extract(remote);
}

int main(int argc, char** argv) {
    try {
        if (argc != 5) throw std::runtime_error("Usage: a2_policy_probe deploy.yaml policy.onnx frames.yaml result.yaml");
        const auto frames = YAML::LoadFile(argv[3]);
        auto sensor = std::make_shared<SensorFrame>();
        load_frame(*sensor, frames[0]);
        const auto cfg = YAML::LoadFile(argv[1]);
        a2_controls::SpeedControl speed(cfg["commands"]["base_velocity"]["speed_control"]);
        auto robot = std::make_shared<unitree::BaseArticulation<std::shared_ptr<SensorFrame>>>(
            sensor, [&] { return speed.scales(); });
        isaaclab::ManagerBasedRLEnv env(cfg, robot);
        auto policy = std::make_unique<RecordingPolicy>(argv[2]);
        auto* recording = policy.get();
        env.alg = std::move(policy);
        YAML::Node result;
        for (std::size_t index = 0; index < frames.size(); ++index) {
            const auto frame = frames[index];
            // The constructor already consumed the first packet; filter each one once.
            if (index != 0) load_frame(*sensor, frame);
            robot->update();
            if (frame["reset"].as<bool>(false)) env.reset();
            if (frame["override_action"]) {
                env.action_manager->process_action(frame["override_action"].as<std::vector<float>>());
            }
            YAML::Node sample;
            sample["targets_before_step"] = env.action_manager->processed_actions();
            env.step();
            sample["obs"] = recording->last_obs;
            sample["raw_action"] = env.action_manager->action();
            sample["policy_targets"] = env.action_manager->processed_actions();
            unitree_hg::msg::dds_::LowCmd_ command;
            unitree::write_joint_position_targets(command, robot->data.joint_ids_map,
                                                 env.action_manager->processed_actions());
            std::vector<float> sdk_targets;
            for (int i = 0; i < 12; ++i) sdk_targets.push_back(command.motor_cmd()[i].q());
            sample["sdk_targets"] = sdk_targets;
            result["samples"].push_back(sample);
        }
        // Reject malformed observations before ONNX Runtime reads the input buffer.
        bool rejected_short = false, rejected_nan = false;
        try { env.alg->act({{"obs", std::vector<float>(45)}}); }
        catch (const std::runtime_error&) { rejected_short = true; }
        try { env.alg->act({{"obs", std::vector<float>(225, std::numeric_limits<float>::quiet_NaN())}}); }
        catch (const std::runtime_error&) { rejected_nan = true; }
        result["rejected_short_observation"] = rejected_short;
        result["rejected_nonfinite_observation"] = rejected_nan;
        for (float angle : {0.0f, 0.99f, 1.01f, 1.57f}) {
            robot->data.projected_gravity_b = Eigen::Vector3f(std::sin(angle), 0, -std::cos(angle));
            result["bad_orientation"].push_back(isaaclab::mdp::bad_orientation(&env, 1.0f));
        }
        robot->data.projected_gravity_b[2] = std::numeric_limits<float>::quiet_NaN();
        result["nonfinite_orientation_rejected"] = isaaclab::mdp::bad_orientation(&env);
        YAML::Emitter emitter;
        emitter.SetFloatPrecision(9);
        emitter << result;
        std::ofstream(argv[4]) << emitter.c_str();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
