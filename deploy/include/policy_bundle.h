#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

namespace policy_bundle {

inline std::filesystem::path model_path(const std::filesystem::path& repository,
                                        std::filesystem::path model) {
    if (model.is_relative()) model = repository / model;
    if (!std::filesystem::is_regular_file(model) || std::filesystem::file_size(model) == 0)
        throw std::runtime_error("Missing or empty ONNX: " + model.string() +
            ". Run play in edu_rl_lab, then copy its exported policy.onnx to this configured path.");
    return std::filesystem::canonical(model);
}

} // namespace policy_bundle
