#pragma once

#include <mujoco/mujoco.h>

#include <tbai_sdk/publisher.hpp>
#include <tbai_sdk/subscriber.hpp>
#include <tbai_sdk/timer.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>

#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class TbaiBridge
{
public:
    TbaiBridge(mjModel *model, mjData *data)
        : mj_model_(model), mj_data_(data)
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

        _check_sensor();

        if (param::config.print_scene_information == 1) {
            printSceneInformation();
        }

        if (param::config.use_joystick == 1) {
            if (param::config.joystick_type == "xbox") {
                joystick_ = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if (param::config.joystick_type == "switch") {
                joystick_ = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        // Initialize tbai_sdk session (creates Zenoh session on first use)
        (void)tbai::session();

        // Create publisher and subscriber
        state_pub_ = std::make_unique<tbai::Publisher<robot_msgs::LowState>>(param::config.low_state_topic);
        cmd_sub_ = std::make_unique<tbai::PollingSubscriber<robot_msgs::MotorCommands>>(param::config.motor_commands_topic);

        // Pre-allocate motor state vector
        state_msg_.motor_states.resize(num_motor_);
    }

    void start()
    {
        running_ = true;
        thread_ = std::thread([this]() {
            tbai::Rate rate(param::config.bridge_rate);
            while (running_.load()) {
                run();
                rate.sleep();
            }
        });
    }

    void stop()
    {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    ~TbaiBridge()
    {
        stop();
    }

private:
    void run()
    {
        if (!mj_data_) return;

        // Update joystick
        if (joystick_) {
            joystick_->update();
        }

        // Read motor commands and apply to simulation
        auto cmd = cmd_sub_->get();
        if (cmd.has_value()) {
            const auto &commands = cmd->commands;
            int n = std::min(static_cast<int>(commands.size()), num_motor_);
            for (int i = 0; i < n; i++) {
                const auto &m = commands[i];
                mj_data_->ctrl[i] = m.tau +
                                     m.kp * (m.q - mj_data_->sensordata[i]) +
                                     m.kd * (m.dq - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // Publish low state
        for (int i = 0; i < num_motor_; i++) {
            state_msg_.motor_states[i].q = mj_data_->sensordata[i];
            state_msg_.motor_states[i].dq = mj_data_->sensordata[i + num_motor_];
            state_msg_.motor_states[i].tau_est = mj_data_->sensordata[i + 2 * num_motor_];
        }

        // IMU
        if (imu_quat_adr_ >= 0) {
            state_msg_.imu_state.quaternion[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
            state_msg_.imu_state.quaternion[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
            state_msg_.imu_state.quaternion[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
            state_msg_.imu_state.quaternion[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

            float w = state_msg_.imu_state.quaternion[0];
            float x = state_msg_.imu_state.quaternion[1];
            float y = state_msg_.imu_state.quaternion[2];
            float z = state_msg_.imu_state.quaternion[3];

            state_msg_.imu_state.rpy[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            state_msg_.imu_state.rpy[1] = asin(2 * (w * y - z * x));
            state_msg_.imu_state.rpy[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        }

        if (imu_gyro_adr_ >= 0) {
            state_msg_.imu_state.gyroscope[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
            state_msg_.imu_state.gyroscope[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
            state_msg_.imu_state.gyroscope[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
        }

        if (imu_acc_adr_ >= 0) {
            state_msg_.imu_state.accelerometer[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
            state_msg_.imu_state.accelerometer[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
            state_msg_.imu_state.accelerometer[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
        }

        // Foot forces
        for (int i = 0; i < 4; i++) {
            if (foot_force_adr_[i] >= 0) {
                state_msg_.foot_force[i] = static_cast<int16_t>(mj_data_->sensordata[foot_force_adr_[i]]);
            } else {
                state_msg_.foot_force[i] = 0;
            }
        }

        state_pub_->publish(state_msg_);
    }

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };

        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });

        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

    void _check_sensor()
    {
        int sensor_id = -1;

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];

        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];

        const char* foot_force_names[4] = {"FR_foot_force", "FL_foot_force", "RR_foot_force", "RL_foot_force"};
        for (int i = 0; i < 4; i++) {
            sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, foot_force_names[i]);
            if (sensor_id >= 0) {
                foot_force_adr_[i] = mj_model_->sensor_adr[sensor_id];
            }
        }
    }

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor addresses
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;
    int foot_force_adr_[4] = {-1, -1, -1, -1};

    // tbai_sdk communication
    std::unique_ptr<tbai::Publisher<robot_msgs::LowState>> state_pub_;
    std::unique_ptr<tbai::PollingSubscriber<robot_msgs::MotorCommands>> cmd_sub_;
    robot_msgs::LowState state_msg_;

    // Joystick
    std::shared_ptr<GamepadState> joystick_;

    // Bridge thread
    std::atomic<bool> running_{false};
    std::thread thread_;
};
