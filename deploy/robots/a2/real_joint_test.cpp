#include "JointTest.h"
#include "network_config.h"

#include <atomic>
#include <boost/program_options.hpp>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace {
using namespace a2_joint_test;
using Clock = std::chrono::steady_clock;
using LowState = unitree_hg::msg::dds_::LowState_;
constexpr auto kControlPeriod = std::chrono::milliseconds(2);
volatile std::sig_atomic_t interrupted = 0;

void handle_signal(int) { interrupted = 1; }
double seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }

class TerminalKeys {
public:
    TerminalKeys() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0)
            throw std::runtime_error("Run in an interactive terminal for Space / Q / Ctrl-C");
        auto settings = saved_;
        settings.c_lflag &= ~(ICANON | ECHO);
        settings.c_cc[VMIN] = 1;
        settings.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &settings) != 0)
            throw std::runtime_error("Could not set terminal input mode");
    }
    ~TerminalKeys() { tcsetattr(STDIN_FILENO, TCSANOW, &saved_); }
    TerminalKeys(const TerminalKeys&) = delete;
    TerminalKeys& operator=(const TerminalKeys&) = delete;

    int read_key() const {
        pollfd fd{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&fd, 1, 0);
        if (ready < 0 && errno != EINTR) throw std::runtime_error("Terminal poll failed");
        if (ready <= 0) return -1;
        if (fd.revents & (POLLHUP | POLLERR | POLLNVAL)) return 'q';
        if (!(fd.revents & POLLIN)) return -1;
        char key;
        const auto count = ::read(STDIN_FILENO, &key, 1);
        if (count < 0 && errno == EINTR) return -1;
        return count == 1 ? static_cast<unsigned char>(key) : 'q';
    }
private:
    termios saved_{};
};

class RealRobot {
public:
    explicit RealRobot(const Config& cfg) : cfg_(cfg) {}

    ~RealRobot() {
        // This guard runs on normal exit, sensor faults, publishing errors and caught signals.
        if (started_) {
            std::cout << "\nStopping joint test (Passive damping)." << std::endl;
            bool sent = true;
            fill_command(command_, cfg_, Target{cfg_.default_sdk}, true);
            for (int i = 0; i < 50; ++i) {
                try { if (!publisher_->Write(command_)) sent = false; }
                catch (...) { sent = false; }
                std::this_thread::sleep_for(kControlPeriod);
            }
            if (!sent) std::cerr << "Could not publish all Passive packets; check the robot directly.\n";
        }
        if (publisher_) publisher_->CloseChannel();
        if (subscriber_) subscriber_->CloseChannel();
    }

    void connect(const std::string& network, double timeout) {
        unitree::robot::ChannelFactory::Instance()->Init(0, network);
        subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
        const auto state = state_;
        subscriber_->InitChannel([state](const void* message) {
            const auto& msg = *static_cast<const LowState*>(message);
            SensorSample sample{{}, msg.mode_machine(), seconds()};
            for (int i = 0; i < 12; ++i) sample.q[i] = msg.motor_state()[i].q();
            std::lock_guard<std::mutex> lock(state->mutex);
            state->sample = sample;
        }, 0);

        // Observe before creating our publisher, so our own packets cannot trigger this check.
        auto other_controller = std::make_shared<std::atomic<bool>>(false);
        unitree::robot::ChannelSubscriber<LowCmd> monitor("rt/lowcmd");
        monitor.InitChannel([other_controller](const void*) { other_controller->store(true); }, 0);
        const double start = seconds();
        while (!interrupted && seconds() - start < timeout) {
            if (other_controller->load()) break;
            if (seconds() - start >= 1.0 && read()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        monitor.CloseChannel();
        if (interrupted) throw std::runtime_error("Connection interrupted");
        if (other_controller->load())
            throw std::runtime_error("Another rt/lowcmd publisher is active. Stop a2_ctrl / other controllers first.");
        check_sample(read(), seconds());
        publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>("rt/lowcmd");
        publisher_->InitChannel();
    }

    std::optional<SensorSample> read() const {
        std::lock_guard<std::mutex> lock(state_->mutex);
        return state_->sample;
    }

    void write(const Target& target) {
        fill_command(command_, cfg_, target);
        started_ = true;
        if (!publisher_->Write(command_)) throw std::runtime_error("Failed to publish rt/lowcmd");
    }

private:
    struct State {
        std::mutex mutex;
        std::optional<SensorSample> sample;
    };
    const Config& cfg_;
    std::shared_ptr<State> state_ = std::make_shared<State>();
    std::unique_ptr<unitree::robot::ChannelSubscriber<LowState>> subscriber_;
    std::unique_ptr<unitree::robot::ChannelPublisher<LowCmd>> publisher_;
    LowCmd command_{};
    bool started_ = false;
};

void run(RealRobot& robot, const Config& cfg, const Motion& motion, const TerminalKeys& keys) {
    const auto initial = robot.read();
    check_sample(initial, seconds());
    Sequence sequence(cfg, initial->q, seconds(), motion);
    std::string previous_description;
    auto next_tick = Clock::now();
    double next_status = seconds();
    while (!interrupted) {
        const double now = seconds();
        const auto state = robot.read();
        check_sample(state, now);
        const int key = keys.read_key();
        if (key == 'q' || key == 'Q' || key == 27) break;
        if (key == ' ') sequence.next_joint(now);
        const auto target = sequence.sample(now);
        robot.write(target);
        const auto description = sequence.description();
        if (description != previous_description) {
            std::cout << "\r\033[K" << description << std::endl;
            previous_description = description;
        }
        if (now >= next_status && sequence.phase() == Sequence::Phase::Wave) {
            const int motor_id = cfg.motor_ids[sequence.policy_id()];
            std::cout << "\r\033[K  " << joint_label(sequence.policy_id()) << " | motor " << motor_id
                      << std::fixed << std::setprecision(4) << " | offset " << std::showpos
                      << target.q[motor_id] - cfg.default_sdk[motor_id]
                      << " | target " << target.q[motor_id] << " | measured " << state->q[motor_id]
                      << std::noshowpos << " rad | Space: next, Q: stop" << std::flush;
            next_status = now + 0.2;
        }
        next_tick = std::max(next_tick + kControlPeriod, Clock::now());
        std::this_thread::sleep_until(next_tick);
    }
}

int main_impl(int argc, char** argv) {
    namespace po = boost::program_options;
    // Keep the same build/ or bin/ next to config/ layout as a2_ctrl.
    const auto project = std::filesystem::read_symlink("/proc/self/exe").parent_path().parent_path();
    std::string network, order, config_path, fsm_path;
    Motion motion;
    double timeout;
    po::options_description desc("A2 real joint test (C++ Unitree SDK2)");
    desc.add_options()
        ("help,h", "show usage")
        ("network,n", po::value<std::string>(&network), "override config.yaml network.interface")
        ("config", po::value<std::string>(&config_path)->default_value((project / "config/deploy.yaml").string()), "policy deployment YAML")
        ("fsm-config", po::value<std::string>(&fsm_path)->default_value((project / "config/config.yaml").string()), "YAML for network interface and Passive damping")
        ("order", po::value<std::string>(&order)->default_value("edu"), "edu or sdk")
        ("amplitude", po::value<double>(&motion.amplitude)->default_value(0.05), "oscillation amplitude in rad (0 < value <= 0.15)")
        ("period", po::value<double>(&motion.period)->default_value(3.0), "oscillation period in seconds (>=2)")
        ("stand-duration", po::value<double>(&motion.stand_duration)->default_value(5.0), "seconds to blend to default pose (>=3)")
        ("connect-timeout", po::value<double>(&timeout)->default_value(5.0), "LowState connection timeout in seconds (>=1)");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) { std::cout << desc << '\n'; return 0; }
    motion.validate();
    if (order != "edu" && order != "sdk") throw std::runtime_error("--order must be edu or sdk");
    motion.sdk_order = order == "sdk";
    if (!std::isfinite(timeout) || timeout < 1) throw std::runtime_error("--connect-timeout must be finite and >=1");
    const auto cfg = Config::load(config_path, fsm_path);
    network = a2_network::resolve(YAML::LoadFile(fsm_path),
        vm.count("network") ? std::make_optional(network) : std::nullopt);
    Sequence preview(cfg, cfg.default_sdk, 0, motion);
    std::cout << "Config: " << config_path << "\nOrder: " << order << "; amplitude +/-" << motion.amplitude
              << " rad; period " << motion.period << "s\n";
    int number = 0;
    for (int policy_id : preview.order())
        std::cout << "  " << std::setw(2) << number++ << ": " << std::setw(17) << std::left << joint_label(policy_id)
                  << std::right << " | EDU " << std::setw(2) << policy_id << " -> motor " << std::setw(2) << cfg.motor_ids[policy_id] << '\n';
    if (!isatty(STDIN_FILENO)) throw std::runtime_error("Run in an interactive terminal for Space / Q / Ctrl-C");
    std::cout << "Space: return current joint then advance. Q / Esc / Ctrl-C: passive stop.\n";

    TerminalKeys keys;
    std::cout << "Support/suspend the robot; this test has no balancing policy.\n"
              << "The robot must already be in low-level/debug mode. Stop other controllers first.\n"
              << "Connecting to A2 on " << network << "; waiting for LowState..." << std::endl;
    RealRobot robot(cfg);
    robot.connect(network, timeout);
    run(robot, cfg, motion, keys);
    return 0;
}
} // namespace

int main(int argc, char** argv) {
    struct sigaction action{};
    action.sa_handler = handle_signal;
    sigemptyset(&action.sa_mask);
    for (int signal : {SIGINT, SIGTERM, SIGHUP}) sigaction(signal, &action, nullptr);
    try { return main_impl(argc, argv); }
    catch (const std::exception& error) {
        if (interrupted) return 0;
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}
