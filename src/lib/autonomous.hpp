#pragma once

#include "../globals.hpp"
#include "sensing.hpp"
#include "subsystems.hpp"
#include "../display/dashboard.hpp"

namespace auton {
    // DEFINITIONS

    const double ADVNC_MINDIFF = 1; // changes advance_dist tolerence (inches)
    const double ADVNC_MAXDIFF = 12; // changes advance_dist scaling upper bound (inches)
    const double CORR_MAXDIFF = 30; // changes rot correction scaling upper bound (deg)
    const double TURN_MINDIFF = 5; // changes turn tolerence (deg)
    const double TURN_MAXDIFF = 60; // changes turn scaling upper bound (deg)
    const double EASE_TIME = 0.3; // changes time to ease movement to max speed

    // SIMPLE MOVEMENT

    // simple move
    inline void advance(int vel, int turn = 0) {
        flmotor.move_velocity(vel+turn);
        frmotor.move_velocity(vel-turn);
        rlmotor.move_velocity(vel+turn);
        rrmotor.move_velocity(vel-turn);
    }
    inline void move(double lvel, double rvel) {
        flmotor.move_velocity(lvel);
        frmotor.move_velocity(rvel);
        rlmotor.move_velocity(lvel);
        rrmotor.move_velocity(rvel);
    }

    // simple turn
    inline void turn(int rotvel) {
        flmotor.move_velocity(rotvel);
        frmotor.move_velocity(-rotvel);
        rlmotor.move_velocity(rotvel);
        rrmotor.move_velocity(-rotvel);
    }

    // stop
    inline void stop() {
        flmotor.move_velocity(0);
        frmotor.move_velocity(0);
        rlmotor.move_velocity(0);
        rrmotor.move_velocity(0);
    }

    // wait with background processing
    inline void wait(double dt) {
        sens::update();
        while (dt > 0) {
            sens::update();
            dashboard::update();
            dt -= sens::dt;
        }
    }
    inline void wait_until(function<bool()> func) {
        sens::update();
        while (!func()) {
            sens::update();
            dashboard::update();
        }
    }

    // MEASURED MOVEMENT

    // move distance
    inline void advance_time(double vel, double dt, double corr_mult = 0) {
        wait(0.1);
        sens::update();
        while (dt > 0) {
            sens::update();
            dashboard::update();
            dt -= sens::dt;
            double rotdiff = angl_180(sens::rot_trg-sens::rot); // rot correction
            rotdiff = limit_range(rotdiff/CORR_MAXDIFF, -1.0, 1.0);
            rotdiff *= abs(rotdiff);
            advance(vel, rotdiff*WHEEL_RPM*corr_mult); // set movement
        }
        stop();
    }
    inline void advance_dist(double dist, double mult = 0.8, double corr_mult = 0.8) {
        wait(0.1);
        sens::update();
        double t0 = sens::t; // time easing start
        double pos0 = drv::get_avg_ldist(); // initial pos
        double pos1 = pos0; // final pos
        double dpos = pos1-pos0; // pos change
        while (abs(dist-dpos) > ADVNC_MINDIFF) {
            sens::update();
            dashboard::update();
            pos1 = drv::get_avg_ldist(); // final pos
            dpos = pos1-pos0; // pos change
            double distdiff = limit_range((dist-dpos)/ADVNC_MAXDIFF, -1.0, 1.0); // pos diff
            double rotdiff = angl_180(sens::rot_trg-sens::rot); // rot correction
            rotdiff = limit_range(rotdiff/CORR_MAXDIFF, -1.0, 1.0);
            rotdiff *= abs(rotdiff);
            advance( // set movement
                distdiff*WHEEL_RPM*mult * min((sens::t-t0)/EASE_TIME, 1.0),
                rotdiff*WHEEL_RPM*corr_mult * min((sens::t-t0)/EASE_TIME, 1.0)
            );
        }
        stop();
    }

    // turn angle
    inline void turn_to(double heading, int force_direction = 0, double mult = 0.5, double max_time = 1) {
        wait(0.1);
        heading = angl_360(heading);
        sens::update();
        double t0 = sens::t; // time easing start
        while (abs(sens::rot-heading) > TURN_MINDIFF) {
            sens::update();
            dashboard::update();
            double rotdiff = angl_180(heading-sens::rot); // rot diff
            if (force_direction > 0 && rotdiff < 0) {rotdiff += 360;} // force cw
            else if (force_direction < 0 && rotdiff > 0) {rotdiff -= 360;} // force ccw
            rotdiff = limit_range(rotdiff/TURN_MAXDIFF, -1.0, 1.0); // rot diff
            turn(rotdiff*WHEEL_RPM*mult * min((sens::t-t0)/EASE_TIME, 1.0)); // set movement
            if (sens::t-t0 > max_time) {break;}
        }
        stop();
        sens::rot_trg = heading;
    }
    inline void turn_angl(double angle) {
        sens::update();
        turn_to(angl_360(sens::rot+angle));
    }

    // INITIALIZE
    inline bool did_init = false;
    inline bool need_sens_reset = false;
    /*
    Runs at the beginning of autonomous before any route.
    For any initializations that cannot occur during initialize(),
    such as calibrating moving parts.
    */
    inline void init() {
        // sensing
        // if previous calibration is invalidated
        if (need_sens_reset && pros::competition::is_autonomous()) {
            sens::reset();
        }
        need_sens_reset = false;

        // calibrate moving parts
        if (!did_init) { // only calibrate once
            
        }

        // initialize finish
        did_init = true;
    }
}