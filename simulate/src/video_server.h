#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// CameraRenderer – renders a named MuJoCo camera into an offscreen buffer,
// encodes as PNG, and publishes as robot_msgs::ImgFrame over Zenoh.
// ---------------------------------------------------------------------------
class CameraRenderer
{
public:
    CameraRenderer(mjModel *model, mjData *data,
                   std::recursive_mutex &sim_mtx,
                   GLFWwindow *gl_window,
                   const std::string &camera_name,
                   int width, int height, double fps,
                   const std::string &topic)
        : model_(model), data_(data), sim_mtx_(sim_mtx),
          gl_window_(gl_window), camera_name_(camera_name),
          width_(width), height_(height),
          render_interval_(1.0 / fps), topic_(topic), running_(false) {}

    ~CameraRenderer() { stop(); }

    void start()
    {
        running_ = true;
        thread_ = std::thread(&CameraRenderer::renderLoop, this);
    }

    void stop()
    {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
    }

private:
    void renderLoop()
    {
        glfwMakeContextCurrent(gl_window_);

        int cam_id = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
        if (cam_id < 0)
        {
            std::cerr << "[VideoServer] Camera '" << camera_name_
                      << "' not found – video publishing disabled." << std::endl;
            glfwMakeContextCurrent(nullptr);
            return;
        }

        mjvScene scn;
        mjvCamera cam;
        mjvOption opt;
        mjrContext con;

        mjv_defaultScene(&scn);
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjr_defaultContext(&con);

        mjv_makeScene(model_, &scn, 2000);
        mjr_makeContext(model_, &con, mjFONTSCALE_150);

        cam.type = mjCAMERA_FIXED;
        cam.fixedcamid = cam_id;

        mjrRect viewport = {0, 0, width_, height_};
        std::vector<uint8_t> rgb(width_ * height_ * 3);

        tbai::Publisher<robot_msgs::ImgFrame> publisher(topic_);

        std::cout << "[VideoServer] Camera '" << camera_name_
                  << "' " << width_ << "x" << height_
                  << " @ " << (1.0 / render_interval_) << " fps → topic '" << topic_ << "'"
                  << std::endl;

        while (running_)
        {
            auto t0 = std::chrono::steady_clock::now();

            // Update scene from sim state
            {
                std::lock_guard<std::recursive_mutex> lock(sim_mtx_);
                mjv_updateScene(model_, data_, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            }

            // Render + read pixels
            mjr_render(viewport, &scn, &con);
            mjr_readPixels(rgb.data(), nullptr, viewport, &con);

            // Flip vertically (OpenGL bottom-left origin → top-left)
            flipVertical(rgb.data(), width_, height_, 3);

            // Publish raw RGB
            {
                robot_msgs::ImgFrame msg;
                auto now = std::chrono::system_clock::now();
                auto epoch = now.time_since_epoch();
                auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
                auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - secs);
                msg.header.stamp.sec = static_cast<int32_t>(secs.count());
                msg.header.stamp.nanosec = static_cast<uint32_t>(nsecs.count());
                msg.header.frame_id = camera_name_;
                msg.height = height_;
                msg.width = width_;
                msg.encoding = "rgb8";
                msg.is_bigendian = 0;
                msg.step = width_ * 3;
                msg.data = rgb;

                publisher.publish(msg);
            }

            // Sleep to maintain FPS
            auto elapsed = std::chrono::steady_clock::now() - t0;
            auto target = std::chrono::duration<double>(render_interval_);
            if (elapsed < target)
                std::this_thread::sleep_for(target - elapsed);
        }

        mjr_freeContext(&con);
        mjv_freeScene(&scn);
        glfwMakeContextCurrent(nullptr);
    }

    static void flipVertical(uint8_t *data, int w, int h, int channels)
    {
        int stride = w * channels;
        std::vector<uint8_t> row(stride);
        for (int y = 0; y < h / 2; ++y)
        {
            uint8_t *top = data + y * stride;
            uint8_t *bot = data + (h - 1 - y) * stride;
            std::memcpy(row.data(), top, stride);
            std::memcpy(top, bot, stride);
            std::memcpy(bot, row.data(), stride);
        }
    }

    mjModel *model_;
    mjData *data_;
    std::recursive_mutex &sim_mtx_;
    GLFWwindow *gl_window_;
    std::string camera_name_;
    int width_;
    int height_;
    double render_interval_;
    std::string topic_;
    std::atomic<bool> running_;
    std::thread thread_;
};
