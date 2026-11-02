#pragma once

#include <net/if.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace a2_network {

// Select an existing interface without changing the host's network settings.
inline std::string resolve(const YAML::Node& config,
                           const std::optional<std::string>& cli_override = std::nullopt)
{
    std::string name;
    if (cli_override) {
        name = *cli_override;
    } else {
        const auto network = config["network"];
        if (!network.IsMap() || !network["interface"].IsScalar())
            throw std::runtime_error("Set network.interface in config.yaml (or --network)");
        name = network["interface"].as<std::string>();
    }
    if (name.empty())
        throw std::runtime_error("Set network.interface in config.yaml (or --network); it must not be empty");
    if (name.find('\0') != std::string::npos || if_nametoindex(name.c_str()) == 0)
        throw std::runtime_error("Network interface '" + name +
            "' does not exist; check config.yaml network.interface or --network");
    return name;
}

} // namespace a2_network
