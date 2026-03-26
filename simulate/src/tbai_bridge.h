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

class TbaiBridge
{
public:
    TbaiBridge(mjModel *model, mjData *data)
        : mj_model_(model), mj_data_(data)
    {
        num_motor_ = mj_model_->nu;
        
        constexpr int MOTOR_SENSOR_NUM = 3;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

        _check_sensor();

        if (param::config.print_scene_information == 1) {
            printSceneInformation();
        }

        // Initialize tbai_sdk session (creates Zenoh session on first use)
        (void)tbai::session();

        // Create publishers and subscriber
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

        // apply motor commands
        auto cmd = cmd_sub_->get();
        if (cmd.has_value()) {
            const auto &commands = cmd->commands;
            int n = std::min(static_cast<int>(commands.size()), num_motor_);
            for (int i = 0; i < n; i++) {
                
                const auto current_q = mj_data_->sensordata[i];
                const auto current_dq = mj_data_->sensordata[i + num_motor_];
                
                const auto &m = commands[i];
                const auto q_error = m.q - current_q;
                const auto dq_error = m.dq - current_dq;

                const auto torque = m.tau + m.kp * q_error + m.kd * dq_error;
                mj_data_->ctrl[i] = torque;
            }
        }

        // motor states
        for (int i = 0; i < num_motor_; i++) {
            state_msg_.motor_states[i].q = mj_data_->sensordata[i];
            state_msg_.motor_states[i].dq = mj_data_->sensordata[i + num_motor_];
            state_msg_.motor_states[i].tau_est = mj_data_->sensordata[i + 2 * num_motor_];
        }

        // imu quaternion (orientation)
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

        // imu gyroscope (angular velocity)
        if (imu_gyro_adr_ >= 0) {
            state_msg_.imu_state.gyroscope[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
            state_msg_.imu_state.gyroscope[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
            state_msg_.imu_state.gyroscope[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
        }

        // imu acceleration
        if (imu_acc_adr_ >= 0) {
            state_msg_.imu_state.accelerometer[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
            state_msg_.imu_state.accelerometer[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
            state_msg_.imu_state.accelerometer[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
        }

        // Foot forces
        for (int i = 0; i < 4; i++) {
            auto value = foot_force_adr_[i] >= 0 ? mj_data_->sensordata[foot_force_adr_[i]] : 0;
            state_msg_.foot_force[i] = static_cast<int16_t>(value);
        }

        // base link position (optional)
        if (frame_pos_adr_ >= 0) {
            state_msg_.has_position = true;
            state_msg_.position[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
            state_msg_.position[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
            state_msg_.position[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
        }

        // base link velocity (optional)
        if (frame_vel_adr_ >= 0) {
            state_msg_.has_velocity = true;
            state_msg_.velocity[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
            state_msg_.velocity[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
            state_msg_.velocity[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
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

        // imu quaternion (orientation)
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];

        // imu gyroscope (angular velocity)
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];

        // imu acceleration
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];

        // base link position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];

        // base link velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];

        // TODO (lnotspotl): load these from config
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

    // Bridge thread
    std::atomic<bool> running_{false};
    std::thread thread_;
};
