#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <vector>
#include <cstring>

namespace param
{

struct Camera {
    bool enabled = true;
    std::string name;
    std::string topic;
    int width = 640;
    int height = 480;
    double fps = 30.0;
};

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    // Topics
    std::string motor_commands_topic = "rt/motor_commands";
    std::string low_state_topic = "rt/low_state";

    // Bridge rate
    double bridge_rate = 1000.0;

    // RGB Cameras
    bool enable_cameras = true;
    std::vector<Camera> cameras;

    // Depth cameras / PointClouds
    bool enable_depth_cameras = false;
    struct DepthCamera {
        bool enabled = true;
        std::string name;
        std::string topic = "rt/pointcloud";
        int width = 320;
        int height = 240;
        double fps = 10.0;
        int stride = 4;
        float min_distance = 0.1f;
        float max_distance = 2.5f;
    };
    std::vector<DepthCamera> depth_cameras;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();

            if (cfg["motor_commands_topic"])
                motor_commands_topic = cfg["motor_commands_topic"].as<std::string>();
            if (cfg["low_state_topic"])
                low_state_topic = cfg["low_state_topic"].as<std::string>();
            if (cfg["bridge_rate"])
                bridge_rate = cfg["bridge_rate"].as<double>();
            // Cameras
            if (cfg["enable_cameras"])
                enable_cameras = cfg["enable_cameras"].as<int>() != 0;
            if (cfg["cameras"]) {
                for (const auto &cam : cfg["cameras"]) {
                    Camera c;
                    c.name = cam["name"].as<std::string>();
                    c.topic = cam["topic"].as<std::string>();
                    if (cam["enabled"]) c.enabled = cam["enabled"].as<int>() != 0;
                    if (cam["width"]) c.width = cam["width"].as<int>();
                    if (cam["height"]) c.height = cam["height"].as<int>();
                    if (cam["fps"]) c.fps = cam["fps"].as<double>();
                    cameras.push_back(c);
                }
            }

            if (cfg["enable_depth_cameras"])
                enable_depth_cameras = cfg["enable_depth_cameras"].as<int>() != 0;
            if (cfg["depth_cameras"]) {
                for (const auto &dc : cfg["depth_cameras"]) {
                    DepthCamera d;
                    d.name = dc["name"].as<std::string>();
                    d.topic = dc["topic"].as<std::string>();
                    if (dc["enabled"]) d.enabled = dc["enabled"].as<int>() != 0;
                    if (dc["width"]) d.width = dc["width"].as<int>();
                    if (dc["height"]) d.height = dc["height"].as<int>();
                    if (dc["fps"]) d.fps = dc["fps"].as<double>();
                    if (dc["stride"]) d.stride = dc["stride"].as<int>();
                    if (dc["min_distance"]) d.min_distance = dc["min_distance"].as<float>();
                    if (dc["max_distance"]) d.max_distance = dc["max_distance"].as<float>();
                    depth_cameras.push_back(d);
                }
            }
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
                      << "Usage: tbai_mujoco <config.yaml> [options]\n"
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
