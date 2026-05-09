#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// DepthImagePublisher – renders depth from a named MuJoCo camera and publishes
// it as a float32 single-channel ImgFrame (encoding "32FC1") over Zenoh.
//
// This is the image-aligned counterpart of PointCloudPublisher: same depth
// values, but the row-major H x W float buffer preserves the pixel ↔ depth
// correspondence (no NaN-stripping, no flattening).
// ---------------------------------------------------------------------------
class DepthImagePublisher
{
public:
    DepthImagePublisher(mjModel *model, mjData *data,
                        std::recursive_mutex &sim_mtx,
                        GLFWwindow *gl_window,
                        const std::string &camera_name,
                        int width, int height, double fps,
                        const std::string &topic = "rt/camera/depth",
                        float min_distance = 0.1f,
                        float max_distance = 5.0f)
        : model_(model), data_(data), sim_mtx_(sim_mtx),
          gl_window_(gl_window), camera_name_(camera_name),
          width_(width), height_(height),
          publish_interval_(1.0 / fps), running_(false), topic_(topic),
          min_distance_(min_distance), max_distance_(max_distance) {}

    ~DepthImagePublisher() { stop(); }

    void start()
    {
        running_ = true;
        thread_ = std::thread(&DepthImagePublisher::publishLoop, this);
    }

    void stop()
    {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
    }

private:
    void publishLoop()
    {
        glfwMakeContextCurrent(gl_window_);

        int cam_id = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
        if (cam_id < 0)
        {
            std::cerr << "[DepthImage] Camera '" << camera_name_
                      << "' not found – depth-image publishing disabled." << std::endl;
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
        std::vector<float> depth_buf(width_ * height_);

        // Clipping planes -- needed to convert OpenGL non-linear depth to metric.
        float extent = static_cast<float>(model_->stat.extent);
        float znear = static_cast<float>(model_->vis.map.znear) * extent;
        float zfar = static_cast<float>(model_->vis.map.zfar) * extent;

        tbai::Publisher<robot_msgs::ImgFrame> publisher(topic_);

        std::cerr << "[DepthImage] Camera '" << camera_name_
                  << "' depth " << width_ << "x" << height_
                  << " @ " << (1.0 / publish_interval_) << " fps -> topic '" << topic_ << "'"
                  << " distance=[" << min_distance_ << ", " << max_distance_ << "]"
                  << "  encoding=32FC1" << std::endl;
        std::cerr.flush();

        const uint32_t step = static_cast<uint32_t>(width_ * sizeof(float));
        std::vector<float> metric_depth(static_cast<size_t>(width_) * height_);

        while (running_)
        {
            auto t0 = std::chrono::steady_clock::now();

            // Update scene
            {
                std::lock_guard<std::recursive_mutex> lock(sim_mtx_);
                mjv_updateScene(model_, data_, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            }

            // Render depth
            mjr_render(viewport, &scn, &con);
            mjr_readPixels(nullptr, depth_buf.data(), viewport, &con);

            // Convert OpenGL depth -> metric, with vertical flip so v=0 is the top
            // row (image-coordinate convention). Out-of-range pixels are written
            // as NaN so downstream consumers can skip them.
            for (int v = 0; v < height_; ++v)
            {
                float *row_out = metric_depth.data() + static_cast<size_t>(v) * width_;
                const float *row_in = depth_buf.data() + static_cast<size_t>(height_ - 1 - v) * width_;
                for (int u = 0; u < width_; ++u)
                {
                    float d = row_in[u];
                    if (d >= 1.0f)
                    {
                        row_out[u] = std::numeric_limits<float>::quiet_NaN();
                        continue;
                    }
                    float z = znear * zfar / (zfar - d * (zfar - znear));
                    if (z < min_distance_ || z > max_distance_)
                    {
                        row_out[u] = std::numeric_limits<float>::quiet_NaN();
                    }
                    else
                    {
                        row_out[u] = z;
                    }
                }
            }

            robot_msgs::ImgFrame msg;
            auto now = std::chrono::system_clock::now();
            auto epoch = now.time_since_epoch();
            auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
            auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - secs);
            msg.header.stamp.sec = static_cast<int32_t>(secs.count());
            msg.header.stamp.nanosec = static_cast<uint32_t>(nsecs.count());
            msg.header.frame_id = camera_name_;

            msg.height = static_cast<uint32_t>(height_);
            msg.width = static_cast<uint32_t>(width_);
            msg.encoding = "32FC1";
            msg.is_bigendian = 0;
            msg.step = step;
            msg.data.resize(static_cast<size_t>(step) * height_);
            std::memcpy(msg.data.data(), metric_depth.data(), msg.data.size());

            publisher.publish(msg);

            auto elapsed = std::chrono::steady_clock::now() - t0;
            auto target = std::chrono::duration<double>(publish_interval_);
            if (elapsed < target)
                std::this_thread::sleep_for(target - elapsed);
        }

        mjr_freeContext(&con);
        mjv_freeScene(&scn);
        glfwMakeContextCurrent(nullptr);
    }

    mjModel *model_;
    mjData *data_;
    std::recursive_mutex &sim_mtx_;
    GLFWwindow *gl_window_;
    std::string camera_name_;
    int width_;
    int height_;
    double publish_interval_;
    std::atomic<bool> running_;
    std::thread thread_;
    std::string topic_;
    float min_distance_;
    float max_distance_;
};
