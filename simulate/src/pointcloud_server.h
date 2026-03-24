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
// PointCloudPublisher – renders depth from a named MuJoCo camera, converts to
// 3D points, and publishes as robot_msgs::PointCloud2 over Zenoh.
// ---------------------------------------------------------------------------
class PointCloudPublisher
{
public:
    PointCloudPublisher(mjModel *model, mjData *data,
                        std::recursive_mutex &sim_mtx,
                        GLFWwindow *gl_window,
                        const std::string &camera_name,
                        int width, int height, double fps,
                        int stride = 4,
                        const std::string &topic = "rt/pointcloud",
                        float min_distance = 0.1f,
                        float max_distance = 2.5f)
        : model_(model), data_(data), sim_mtx_(sim_mtx),
          gl_window_(gl_window), camera_name_(camera_name),
          width_(width), height_(height), stride_(std::max(stride, 1)),
          publish_interval_(1.0 / fps), running_(false), topic_(topic),
          min_distance_(min_distance), max_distance_(max_distance) {}

    ~PointCloudPublisher() { stop(); }

    void start()
    {
        running_ = true;
        thread_ = std::thread(&PointCloudPublisher::publishLoop, this);
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
            std::cerr << "[PointCloud] Camera '" << camera_name_
                      << "' not found – pointcloud publishing disabled." << std::endl;
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

        // Camera intrinsics from MuJoCo camera fovy
        double fovy_rad = model_->cam_fovy[cam_id] * M_PI / 180.0;
        double fy = static_cast<double>(height_) / (2.0 * std::tan(fovy_rad / 2.0));
        double fx = fy;
        double cx = static_cast<double>(width_) / 2.0;
        double cy = static_cast<double>(height_) / 2.0;

        // Clipping planes
        float extent = static_cast<float>(model_->stat.extent);
        float znear = static_cast<float>(model_->vis.map.znear) * extent;
        float zfar = static_cast<float>(model_->vis.map.zfar) * extent;

        tbai::Publisher<robot_msgs::PointCloud2> publisher(topic_);

        // Pre-build PointField descriptors (x, y, z as FLOAT32 = datatype 7)
        constexpr uint8_t FLOAT32 = 7;
        constexpr uint32_t point_step = 12; // 3 * sizeof(float)

        int sampled_w = (width_ + stride_ - 1) / stride_;
        int sampled_h = (height_ + stride_ - 1) / stride_;

        std::cout << "[PointCloud] Camera '" << camera_name_
                  << "' depth " << width_ << "x" << height_
                  << " stride=" << stride_
                  << " (" << sampled_w << "x" << sampled_h << " sampled)"
                  << " @ " << (1.0 / publish_interval_) << " fps → topic '" << topic_ << "'"
                  << " distance=[" << min_distance_ << ", " << max_distance_ << "]" << std::endl;

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

            // Convert to point cloud
            std::vector<uint8_t> cloud_data;
            cloud_data.reserve(sampled_w * sampled_h * point_step);

            uint32_t valid_points = 0;
            for (int v = 0; v < height_; v += stride_)
            {
                for (int u = 0; u < width_; u += stride_)
                {
                    float d = depth_buf[(height_ - 1 - v) * width_ + u];
                    if (d >= 1.0f) continue;

                    float z = znear * zfar / (zfar - d * (zfar - znear));
                    if (z < min_distance_ || z > max_distance_) continue;

                    float x = static_cast<float>((static_cast<double>(u) - cx) * z / fx);
                    float y = static_cast<float>((static_cast<double>(v) - cy) * z / fy);

                    size_t offset = cloud_data.size();
                    cloud_data.resize(offset + point_step);
                    std::memcpy(cloud_data.data() + offset + 0, &x, sizeof(float));
                    std::memcpy(cloud_data.data() + offset + 4, &y, sizeof(float));
                    std::memcpy(cloud_data.data() + offset + 8, &z, sizeof(float));
                    ++valid_points;
                }
            }

            // Build PointCloud2 message
            robot_msgs::PointCloud2 msg;

            auto now = std::chrono::system_clock::now();
            auto epoch = now.time_since_epoch();
            auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
            auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - secs);
            msg.header.stamp.sec = static_cast<int32_t>(secs.count());
            msg.header.stamp.nanosec = static_cast<uint32_t>(nsecs.count());
            msg.header.frame_id = camera_name_;

            msg.height = 1;
            msg.width = valid_points;
            msg.fields = {
                {"x", 0, FLOAT32, 1},
                {"y", 4, FLOAT32, 1},
                {"z", 8, FLOAT32, 1},
            };
            msg.is_bigendian = 0;
            msg.point_step = point_step;
            msg.row_step = valid_points * point_step;
            msg.data = std::move(cloud_data);
            msg.is_dense = 1;

            publisher.publish(msg);

            // Sleep to maintain FPS
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
    int stride_;
    double publish_interval_;
    std::atomic<bool> running_;
    std::thread thread_;
    std::string topic_;
    float min_distance_;
    float max_distance_;
};
