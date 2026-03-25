#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <cstring>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    // Topics
    std::string motor_commands_topic = "rt/motor_commands";
    std::string low_state_topic = "rt/low_state";

    // Bridge rate
    double bridge_rate = 1000.0;

    // Camera / Video
    bool enable_camera = false;
    std::string camera_name;
    int camera_width = 640;
    int camera_height = 480;
    double camera_fps = 30.0;
    std::string camera_topic = "rt/camera/image";

    // Depth camera / PointCloud
    bool enable_depth_camera = false;
    std::string depth_camera_name;
    int depth_camera_width = 320;
    int depth_camera_height = 240;
    double depth_camera_fps = 10.0;
    int depth_camera_stride = 4;
    float depth_camera_min_distance = 0.1f;
    float depth_camera_max_distance = 2.5f;
    std::string pointcloud_topic = "rt/pointcloud";

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();

            if (cfg["motor_commands_topic"])
                motor_commands_topic = cfg["motor_commands_topic"].as<std::string>();
            if (cfg["low_state_topic"])
                low_state_topic = cfg["low_state_topic"].as<std::string>();
            if (cfg["bridge_rate"])
                bridge_rate = cfg["bridge_rate"].as<double>();

            if (cfg["enable_camera"])
                enable_camera = cfg["enable_camera"].as<int>() != 0;
            if (cfg["camera_name"])
                camera_name = cfg["camera_name"].as<std::string>();
            if (cfg["camera_width"])
                camera_width = cfg["camera_width"].as<int>();
            if (cfg["camera_height"])
                camera_height = cfg["camera_height"].as<int>();
            if (cfg["camera_fps"])
                camera_fps = cfg["camera_fps"].as<double>();
            if (cfg["camera_topic"])
                camera_topic = cfg["camera_topic"].as<std::string>();

            if (cfg["enable_depth_camera"])
                enable_depth_camera = cfg["enable_depth_camera"].as<int>() != 0;
            if (cfg["depth_camera_name"])
                depth_camera_name = cfg["depth_camera_name"].as<std::string>();
            if (cfg["depth_camera_width"])
                depth_camera_width = cfg["depth_camera_width"].as<int>();
            if (cfg["depth_camera_height"])
                depth_camera_height = cfg["depth_camera_height"].as<int>();
            if (cfg["depth_camera_fps"])
                depth_camera_fps = cfg["depth_camera_fps"].as<double>();
            if (cfg["depth_camera_stride"])
                depth_camera_stride = cfg["depth_camera_stride"].as<int>();
            if (cfg["depth_camera_min_distance"])
                depth_camera_min_distance = cfg["depth_camera_min_distance"].as<float>();
            if (cfg["depth_camera_max_distance"])
                depth_camera_max_distance = cfg["depth_camera_max_distance"].as<float>();
            if (cfg["pointcloud_topic"])
                pointcloud_topic = cfg["pointcloud_topic"].as<std::string>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

// Simple command-line argument parser (no boost dependency)
inline void helper(int argc, char** argv)
{
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << "TBAI MuJoCo Simulator\n"
                      << "Usage: tbai_mujoco [config.yaml] [options]\n"
                      << "  -r, --robot <name>    Robot type (e.g. go2)\n"
                      << "  -s, --scene <file>    Robot scene file\n"
                      << "  -h, --help            Show this help\n";
            exit(0);
        }
        if ((arg == "-r" || arg == "--robot") && i + 1 < argc) {
            config.robot = argv[++i];
        }
        if ((arg == "-s" || arg == "--scene") && i + 1 < argc) {
            config.robot_scene = argv[++i];
        }
    }
}

}
