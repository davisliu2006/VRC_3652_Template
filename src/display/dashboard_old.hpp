#pragma once

#include "../globals.hpp"
#include "core.hpp"
#include "../lib/sensing.hpp"

namespace dashboard {
    using namespace display;

    //DEFINITIONS

    const double delay = 1;
    inline double displ_time = 0;

    // motor temperatures
    inline vector<vector<tuple<pros::Motor, string>>> temp_data = {
        {{flmotor, "FL"}, {frmotor, "FR"}},
        {{rlmotor, "RL"}, {rrmotor, "RR"}}
    };
    inline void displ_temp() {
        display::update();
        pros::screen::set_pen(RGB2COLOR(255, 255, 255));
        for (int i = 0; i < temp_data.size(); i++) {
            auto& line = temp_data[i];
            string txt = "";
            for (int j = 0; j < line.size(); j++) {
                auto& [mtr, name] = line[j];
                txt += name+": "+to_string(int(mtr.get_temperature()))+"°C";
                txt += (j == line.size()-1? '\n' : '\t');
            }
            pros::screen::print(pros::E_TEXT_SMALL, 5, 10+i*20, txt.c_str());
        }
    }

    //EVENTS

    // initialize
    inline void init() {}

    // enable and disable
    inline void enable() {}
    inline void disable() {}
    
    // update
    /*
    Call this every frame to update and display the dashboard.
    */
    inline void update() {
        if (displ_time >= delay) {
            displ_temp();
            displ_time -= delay;
        } else {
            displ_time += sens::dt;
        }
    }
}