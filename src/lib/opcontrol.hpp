#pragma once

#include "../globals.hpp"
#include "sensing.hpp"
#include "../display/main.hpp"

// maps joystick position to drive speed
inline double joymap(int x){
    double temp = double(x)/MTR_MAX;
    return temp*sqrt(abs(temp));
}

// start opcontrol
inline void opcontrol_start() {
    // drivetrain
    bool drv_rev = 1; // reverse drivetrain: set to -1
    bool drv_rev_pressed = false;

    while (true) {
        // sensing
        sens::update();

        // display
        dashboard::update();

        // drivetrain
        double x = joymap(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X))*drv_rev;
        double y = joymap(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))*drv_rev;
        double rot = joymap(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        flmotor.move_velocity((y+rot)*WHEEL_RPM);
        frmotor.move_velocity((y-rot)*WHEEL_RPM);
        rlmotor.move_velocity((y+rot)*WHEEL_RPM);
        rrmotor.move_velocity((y-rot)*WHEEL_RPM);
    }
}