#pragma once

#include <iostream>
#include <memory>
#include "joystick/joystick.h"

// Standalone joystick abstraction (no unitree_sdk2 dependency)
class GamepadState
{
public:
    virtual ~GamepadState() = default;
    virtual void update() = 0;

    // Buttons
    bool back_ = false, start_ = false;
    bool LB_ = false, RB_ = false;
    bool A_ = false, B_ = false, X_ = false, Y_ = false;
    bool up_ = false, down_ = false, left_ = false, right_ = false;
    bool LT_ = false, RT_ = false;

    // Axes (normalized to [-1, 1])
    double lx_ = 0, ly_ = 0, rx_ = 0, ry_ = 0;
};

class XBoxJoystick : public GamepadState
{
public:
    XBoxJoystick(std::string device, int bits = 15)
    {
        js_ = std::make_unique<Joystick>(device);
        if(!js_->isFound()) {
            std::cout << "Error: Joystick open failed." << std::endl;
            exit(1);
        }
        max_value_ = 1 << (bits - 1);
    }

    void update() override
    {
        js_->getState();
        back_ = js_->button_[6];
        start_ = js_->button_[7];
        LB_ = js_->button_[4];
        RB_ = js_->button_[5];
        A_ = js_->button_[0];
        B_ = js_->button_[1];
        X_ = js_->button_[2];
        Y_ = js_->button_[3];
        up_ = js_->axis_[7] < 0;
        down_ = js_->axis_[7] > 0;
        left_ = js_->axis_[6] < 0;
        right_ = js_->axis_[6] > 0;
        LT_ = js_->axis_[2] > 0;
        RT_ = js_->axis_[5] > 0;
        lx_ = double(js_->axis_[0]) / max_value_;
        ly_ = -double(js_->axis_[1]) / max_value_;
        rx_ = double(js_->axis_[3]) / max_value_;
        ry_ = -double(js_->axis_[4]) / max_value_;
    }
private:
    std::unique_ptr<Joystick> js_;
    int max_value_;
};


class SwitchJoystick : public GamepadState
{
public:
    SwitchJoystick(std::string device, int bits = 15)
    {
        js_ = std::make_unique<Joystick>(device);
        if(!js_->isFound()) {
            std::cout << "Error: Joystick open failed." << std::endl;
            exit(1);
        }
        max_value_ = 1 << (bits - 1);
    }

    void update() override
    {
        js_->getState();
        back_ = js_->button_[10];
        start_ = js_->button_[11];
        LB_ = js_->button_[6];
        RB_ = js_->button_[7];
        A_ = js_->button_[0];
        B_ = js_->button_[1];
        X_ = js_->button_[3];
        Y_ = js_->button_[4];
        up_ = js_->axis_[7] < 0;
        down_ = js_->axis_[7] > 0;
        left_ = js_->axis_[6] < 0;
        right_ = js_->axis_[6] > 0;
        LT_ = js_->axis_[5] > 0;
        RT_ = js_->axis_[4] > 0;
        lx_ = double(js_->axis_[0]) / max_value_;
        ly_ = -double(js_->axis_[1]) / max_value_;
        rx_ = double(js_->axis_[2]) / max_value_;
        ry_ = -double(js_->axis_[3]) / max_value_;
    }
private:
    std::unique_ptr<Joystick> js_;
    int max_value_;
};
