#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

namespace policy_bundle {

inline std::filesystem::path resolve(const std::filesystem::path& repository,
                                     std::filesystem::path directory) {
    if (directory.is_relative()) directory = repository / directory;
    if (!std::filesystem::is_directory(directory))
        throw std::runtime_error("Policy folder not found: " + directory.string());
    return std::filesystem::canonical(directory);
}

inline std::filesystem::path model_path(const std::filesystem::path& repository,
                                        const std::filesystem::path& directory) {
    const auto policy_dir = resolve(repository, directory);
    const auto onnx = policy_dir / "policy.onnx";
    if (!std::filesystem::is_regular_file(onnx) || std::filesystem::file_size(onnx) == 0)
        throw std::runtime_error("Missing or empty ONNX: " + onnx.string() +
            ". Run play in edu_rl_lab, then copy its exported policy.onnx into this folder.");
    return onnx;
}

} // namespace policy_bundle
