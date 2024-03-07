#pragma once

#include "../globals.hpp"

// drivetrain
namespace drv {
    inline double get_avg_position() {
        return (flmotor.get_position()+frmotor.get_position()
            +rlmotor.get_position()+rrmotor.get_position())/4;
    }
    inline double get_avg_ldist() {
        return get_avg_position()/360*WHEEL_C;
    }
}