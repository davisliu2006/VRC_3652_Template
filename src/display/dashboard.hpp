#pragma once

#include "../globals.hpp"
#include "core.hpp"
#include "../lib/sensing.hpp"

namespace dashboard {
    using namespace display;

    //DEFINITIONS

    const double DELAY = 1;
    inline double displ_time = 0;

    // lines to display
    inline vector<vector<function<string()>>> display_lines = {
        // drivetrain
        {
            []() {return "FL: "+to_string(int(flmotor.get_temperature()))+"°C";},
            []() {return "FR: "+to_string(int(frmotor.get_temperature()))+"°C";},
            
        },
        {
            []() {return "RL: "+to_string(int(rlmotor.get_temperature()))+"°C";},
            []() {return "RR: "+to_string(int(rrmotor.get_temperature()))+"°C";},
        },
        // sensing
        {
            []() {return "Rot: "+to_string(sens::rot);},
            []() {return "Trg: "+to_string(sens::rot_trg);}
        }
    };

    //EVENTS

    // initialize
    inline void init() {}
    
    // update
    /*
    Call this every frame to update and display the dashboard.
    */
    inline void update() {
        if (displ_time >= DELAY) {
            displ_time -= DELAY;
            display::update();
            for (int i = 0; i < display_lines.size(); i++) {
                auto& line = display_lines[i];
                string txt;
                for (int j = 0; j < line.size(); j++) {
                    txt += line[j]();
                    if (j != line.size()-1) {txt += "  ";}
                }
                pros::screen::print(pros::E_TEXT_SMALL, 5, 10+i*20, txt.c_str());
            }
        } else {
            displ_time += sens::dt;
        }
    }
}