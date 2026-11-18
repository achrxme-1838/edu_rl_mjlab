#pragma once

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace a2_config {

// Read the single Python source at startup, before any DDS channels/threads.
// Use the mjlab interpreter selected at build time, or A2_PYTHON at runtime.
// No shell, temporary files, or cached configuration copies.
inline YAML::Node load(const std::filesystem::path& repository,
                       const std::filesystem::path& config = {}) {
    const auto adapter = repository / "deploy/configs/a2/runtime.py";
    const auto source = config.empty() ? repository / "deploy/configs/a2/config.py"
                                       : std::filesystem::absolute(config);
    if (!std::filesystem::is_regular_file(adapter) || !std::filesystem::is_regular_file(source))
        throw std::runtime_error("Missing A2 Python configuration: " + source.string());
    const auto adapter_arg = adapter.string();
    const auto source_arg = source.string();
    const auto override_python = std::getenv("A2_PYTHON");
    const std::string python = override_python ? override_python : A2_CONFIG_PYTHON;
    if (python.empty() || !std::filesystem::path(python).is_absolute())
        throw std::runtime_error("A2_PYTHON must be an absolute path to Python with mjlab installed");
    int output[2];
    if (pipe(output) != 0) throw std::runtime_error("Cannot create Python config pipe");
    const auto pid = fork();
    if (pid < 0) {
        close(output[0]); close(output[1]);
        throw std::runtime_error("Cannot start Python config loader");
    }
    if (pid == 0) {
        close(output[0]);
        if (dup2(output[1], STDOUT_FILENO) < 0) _exit(126);
        close(output[1]);
        execl(python.c_str(), python.c_str(), "-I", "-B", adapter_arg.c_str(), source_arg.c_str(), nullptr);
        _exit(127);
    }
    close(output[1]);
    std::string document;
    char buffer[4096];
    bool read_failed = false;
    while (true) {
        const auto count = read(output[0], buffer, sizeof(buffer));
        if (count > 0) document.append(buffer, static_cast<std::size_t>(count));
        else if (count == 0) break;
        else if (errno != EINTR) { read_failed = true; break; }
    }
    close(output[0]);
    int status = 0;
    pid_t waited;
    do { waited = waitpid(pid, &status, 0); } while (waited < 0 && errno == EINTR);
    if (read_failed || waited < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
        throw std::runtime_error("Cannot load " + source_arg +
            " using " + python + "; check the Python error above and install mjlab in this environment,"
            " or set A2_PYTHON to the correct Python executable");
    const auto cfg = YAML::Load(document);
    if (!cfg.IsMap() || !cfg["deploy"].IsMap() || !cfg["FSM"].IsMap())
        throw std::runtime_error("Python config loader returned an invalid runtime document");
    return cfg;
}

} // namespace a2_config
