// Repeatedly stand up and sit down — using tbai_sdk over Zenoh.
// Usage: stand_go2 [hold_duration_s] [ramp_duration_s] [save_every_nth_frame]

#include <algorithm>
#include <deque>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/subscriber.hpp>
#include <tbai_sdk/timer.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>

#include "lodepng.h"

static constexpr int NUM_MOTORS = 12;
static constexpr double DT = 0.002;       // 500 Hz
static constexpr double RAMP_TAU = 1.2;   // tanh time constant for transitions

// Actuator order: FR(hip,thigh,calf), FL, RR, RL
static constexpr double STAND_UP[NUM_MOTORS] = {
     0.00572,  0.60881, -1.21763,
    -0.00572,  0.60881, -1.21763,
     0.00572,  0.60881, -1.21763,
    -0.00572,  0.60881, -1.21763,
};

static constexpr double STAND_DOWN[NUM_MOTORS] = {
     0.04735,  1.22187, -2.44375,
    -0.04735,  1.22187, -2.44375,
     0.04735,  1.22187, -2.44375,
    -0.04735,  1.22187, -2.44375,
};

// Turbo-ish colormap: maps 0..1 → RGB
static void depth_to_rgb(float norm, uint8_t &r, uint8_t &g, uint8_t &b)
{
    // clamp
    norm = std::max(0.0f, std::min(1.0f, norm));
    // 5-segment turbo-like colormap
    if (norm < 0.25f) {
        float t = norm / 0.25f;
        r = 0;
        g = static_cast<uint8_t>(t * 255);
        b = 255;
    } else if (norm < 0.5f) {
        float t = (norm - 0.25f) / 0.25f;
        r = 0;
        g = 255;
        b = static_cast<uint8_t>((1.0f - t) * 255);
    } else if (norm < 0.75f) {
        float t = (norm - 0.5f) / 0.25f;
        r = static_cast<uint8_t>(t * 255);
        g = 255;
        b = 0;
    } else {
        float t = (norm - 0.75f) / 0.25f;
        r = 255;
        g = static_cast<uint8_t>((1.0f - t) * 255);
        b = 0;
    }
}

// Reconstruct a depth heatmap image from a PointCloud2 message.
// Returns true if a valid image was produced.
static bool pointcloud_to_depth_png(const robot_msgs::PointCloud2 &pc,
                                    int img_w, int img_h,
                                    float fov_y_deg, float max_depth,
                                    std::vector<uint8_t> &png_out)
{
    if (pc.data.empty() || pc.point_step < 12)
        return false;

    // Camera intrinsics (must match the depth camera used to produce the cloud)
    double fovy_rad = fov_y_deg * M_PI / 180.0;
    double fy = static_cast<double>(img_h) / (2.0 * std::tan(fovy_rad / 2.0));
    double fx = fy;
    double cx = static_cast<double>(img_w) / 2.0;
    double cy = static_cast<double>(img_h) / 2.0;

    // Depth buffer (per-pixel, initialized to max)
    std::vector<float> depth_map(img_w * img_h, max_depth);

    // Reproject each 3D point back to image coordinates
    uint32_t num_points = pc.width * pc.height;
    for (uint32_t i = 0; i < num_points; i++)
    {
        float x, y, z;
        const uint8_t *ptr = pc.data.data() + i * pc.point_step;
        std::memcpy(&x, ptr + 0, 4);
        std::memcpy(&y, ptr + 4, 4);
        std::memcpy(&z, ptr + 8, 4);

        if (z <= 0.0f) continue;

        int u = static_cast<int>(x * fx / z + cx);
        int v = static_cast<int>(y * fy / z + cy);

        if (u < 0 || u >= img_w || v < 0 || v >= img_h) continue;

        // Keep closest point per pixel
        float &cur = depth_map[v * img_w + u];
        if (z < cur) cur = z;
    }

    // Nearest-neighbor fill: for each empty pixel, find closest filled pixel
    // using BFS from all filled pixels simultaneously
    {
        const float EMPTY = max_depth;
        std::vector<bool> visited(img_w * img_h, false);
        std::deque<int> queue;

        // Seed with all filled pixels
        for (int i = 0; i < img_w * img_h; i++) {
            if (depth_map[i] < EMPTY) {
                visited[i] = true;
                queue.push_back(i);
            }
        }

        const int dx[] = {-1, 1, 0, 0};
        const int dy[] = {0, 0, -1, 1};

        while (!queue.empty()) {
            int idx = queue.front();
            queue.pop_front();
            int py = idx / img_w;
            int px = idx % img_w;

            for (int d = 0; d < 4; d++) {
                int nx = px + dx[d];
                int ny = py + dy[d];
                if (nx < 0 || nx >= img_w || ny < 0 || ny >= img_h) continue;
                int nidx = ny * img_w + nx;
                if (visited[nidx]) continue;
                visited[nidx] = true;
                depth_map[nidx] = depth_map[idx];
                queue.push_back(nidx);
            }
        }
    }

    // Render heatmap as RGB
    std::vector<uint8_t> rgb(img_w * img_h * 3);
    for (int i = 0; i < img_w * img_h; i++)
    {
        float norm = depth_map[i] / max_depth;
        depth_to_rgb(norm, rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
    }

    unsigned err = lodepng::encode(png_out, rgb.data(),
                                   static_cast<unsigned>(img_w),
                                   static_cast<unsigned>(img_h),
                                   LCT_RGB, 8);
    return err == 0;
}

int main(int argc, char** argv)
{
    double hold_duration = 2.0;
    double ramp_duration = 3.0 * RAMP_TAU;  // default ~3.6s
    int save_every_n = 10;                   // save every Nth frame
    if (argc > 1)
        hold_duration = std::atof(argv[1]);
    if (argc > 2)
        ramp_duration = std::atof(argv[2]);
    if (argc > 3)
        save_every_n = std::max(1, std::atoi(argv[3]));

    double tau = ramp_duration / 3.0;
    double half_cycle = ramp_duration + hold_duration;
    double full_cycle = 2.0 * half_cycle;

    // Create output directories
    std::filesystem::create_directories("images");
    std::filesystem::create_directories("depth");

    std::printf("Stand Go2 — hold=%.1fs  ramp=%.1fs  cycle=%.1fs  save_every=%d\n",
                hold_duration, ramp_duration, full_cycle, save_every_n);
    std::printf("Saving RGB to images/  and depth heatmaps to depth/\n");
    std::printf("Press ENTER to start...");
    std::cin.get();

    tbai::Publisher<robot_msgs::MotorCommands> cmd_pub("rt/motor_commands");
    tbai::PollingSubscriber<robot_msgs::LowState> state_sub("rt/low_state");
    tbai::PollingSubscriber<robot_msgs::ImgFrame> image_sub("rt/camera/image");
    tbai::PollingSubscriber<robot_msgs::PointCloud2> pc_sub("rt/pointcloud");

    robot_msgs::MotorCommands cmd;
    cmd.commands.resize(NUM_MOTORS);

    double t = 0.0;
    uint64_t last_saved_img = 0;
    uint64_t last_saved_pc = 0;
    int saved_images = 0;
    int saved_depths = 0;
    tbai::Rate rate(1.0 / DT);

    // Depth heatmap parameters (must match config.yaml depth camera settings)
    constexpr int DEPTH_W = 320;
    constexpr int DEPTH_H = 240;
    constexpr float DEPTH_FOV_Y = 87.0f;  // depth_camera fovy from go2.xml
    constexpr float DEPTH_MAX = 2.5f;

    while (true)
    {
        t += DT;

        double cycle_t = std::fmod(t, full_cycle);
        bool standing_up = cycle_t < half_cycle;

        double phase;
        const double* from;
        const double* to;

        if (standing_up) {
            from = STAND_DOWN;
            to   = STAND_UP;
            phase = std::tanh(cycle_t / tau);
        } else {
            from = STAND_UP;
            to   = STAND_DOWN;
            phase = std::tanh((cycle_t - half_cycle) / tau);
        }

        for (int i = 0; i < NUM_MOTORS; i++) {
            cmd.commands[i].q   = (1.0 - phase) * from[i] + phase * to[i];
            cmd.commands[i].dq  = 0.0f;
            cmd.commands[i].kp  = 50.0f;
            cmd.commands[i].kd  = 3.5f;
            cmd.commands[i].tau = 0.0f;
        }

        cmd_pub.publish(cmd);

        // Save every Nth received RGB image (arrives as raw rgb8, encode to PNG at save time)
        uint64_t img_count = image_sub.message_count();
        if (img_count > 0 && img_count >= last_saved_img + save_every_n)
        {
            if (auto img = image_sub.get()) {
                std::vector<uint8_t> png;
                unsigned err = lodepng::encode(png, img->data.data(),
                                               img->width, img->height, LCT_RGB, 8);
                if (err == 0) {
                    char filename[128];
                    std::snprintf(filename, sizeof(filename), "images/%06d.png", saved_images);
                    std::ofstream out(filename, std::ios::binary);
                    out.write(reinterpret_cast<const char*>(png.data()), png.size());
                    std::printf("[%6.1fs] Saved %s (%zu bytes)\n", t, filename, png.size());
                }
                last_saved_img = img_count;
                ++saved_images;
            }
        }

        // Save every Nth received pointcloud as depth heatmap
        uint64_t pc_count = pc_sub.message_count();
        if (pc_count > 0 && pc_count >= last_saved_pc + save_every_n)
        {
            if (auto pc = pc_sub.get()) {
                std::vector<uint8_t> png;
                if (pointcloud_to_depth_png(*pc, DEPTH_W, DEPTH_H, DEPTH_FOV_Y, DEPTH_MAX, png)) {
                    char filename[128];
                    std::snprintf(filename, sizeof(filename), "depth/%06d.png", saved_depths);
                    std::ofstream out(filename, std::ios::binary);
                    out.write(reinterpret_cast<const char*>(png.data()), png.size());
                    std::printf("[%6.1fs] Saved %s (%u pts → %zu bytes)\n",
                                t, filename, pc->width, png.size());
                    last_saved_pc = pc_count;
                    ++saved_depths;
                }
            }
        }

        // Print state once per second
        if (auto st = state_sub.get(); st && static_cast<int>(t / DT) % 500 == 0) {
            std::printf("[%6.1fs] %s  q0=%.3f q1=%.3f q2=%.3f\n",
                        t, standing_up ? "UP  " : "DOWN",
                        st->motor_states[0].q, st->motor_states[1].q, st->motor_states[2].q);
        }

        rate.sleep();
    }
}
