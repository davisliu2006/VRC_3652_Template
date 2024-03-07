#pragma once

#include "../globals.hpp"
#include "dashboard.hpp"
#include "selection.hpp"

namespace display {
    // initialize all displays
    inline void init_all() {
        init();
        selection::init();
        dashboard::init();
    }

    // display to show on initialize()
    inline void on_init() {
        selection::enable();
    }
    // display to show on disabled()
    inline void on_disable() {
        selection::enable();
    }
    // display to show on opcontrol()
    inline void on_opcontrol() {
        selection::disable();
    }
    // display to show on autonomous()
    inline void on_auton() {
        selection::disable();
    }
}