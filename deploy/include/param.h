// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <memory>
#include <iomanip>
#include "policy_bundle.h"
#include "python_config.h"

/* ---------- logger ---------- */
namespace spdlog
{
inline void create_logger(std::string log_path)
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path, 5 * 1024 * 1024, 5);

    std::vector<spdlog::sink_ptr> sinks {console_sink, rotating_sink};
    auto logger = std::make_shared<spdlog::logger>("unitree", sinks.begin(), sinks.end());

    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%^%l%$] %v");
    logger->flush_on(spdlog::level::info);

    spdlog::set_default_logger(logger);
}

} // namespace spdlog


namespace param
{
inline std::string VERSION = "1.0.0.1";
inline std::filesystem::path bin_path;
inline std::filesystem::path proj_dir;
inline std::filesystem::path repo_dir;
inline std::filesystem::path config_path;
inline YAML::Node config;
inline YAML::Node deploy_config;

inline std::filesystem::path get_bin_path() {
    std::vector<char> path(1024);
    ssize_t len = readlink("/proc/self/exe", &path[0], path.size());
    if (len != -1) {
        path[len] = '\0';  // Null-terminate the result
        return std::filesystem::path(&path[0]);
    } else {
        spdlog::error("Failed to get executable path.");
        exit(1);
    }
}

inline void locate_project()
{
    proj_dir = bin_path.parent_path();
    if (proj_dir.filename() == "bin" || proj_dir.filename() == "build")
        proj_dir = proj_dir.parent_path();
    repo_dir = proj_dir.parent_path().parent_path().parent_path();
}

inline void load_deploy_config()
{
    config = a2_config::load(repo_dir, config_path);
    deploy_config = config["deploy"];
}

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv) 
{
    bin_path = get_bin_path();
    locate_project();

    po::options_description desc("Unitree Controller");
    desc.add_options()
        ("help,h", "produce help message")
        ("version,v", "show version")
        ("log", "record log file")
        ("network,n", po::value<std::string>(), "override Python config network.interface")
        ("config", po::value<std::string>(), "Python file defining A2DeployCfg")
        ("check-policy", "validate the copied ONNX policy, then exit without DDS")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }
    if (vm.count("version"))
    {
        std::cout << "Version: " << VERSION << std::endl;
        exit(0);
    }

#ifndef NDEBUG
    spdlog::set_level(spdlog::level::debug);
#else
    spdlog::set_level(spdlog::level::info);
#endif
    if(vm.count("log"))
    {
        std::filesystem::create_directories(proj_dir / "log");
        spdlog::create_logger(proj_dir.string() + "/log/log.txt");
    }

    if (vm.count("config")) config_path = vm["config"].as<std::string>();
    return vm;
}

}
