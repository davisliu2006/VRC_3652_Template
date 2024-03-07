#pragma once

#include "../globals.hpp"
#include "sensing.hpp"
#include "../display/main.hpp"

struct PID {
    const double KP = 0;
    const double KI = 0;
    const double KD = 0;

    double error = 0;
    double integral = 0;
    double derivitive = 0;

    inline void reset() {
        error = 0;
        integral = 0;
        derivitive = 0;
    }
    inline void update(double trg, double curr) {
        sens::update();
        double error0 = error;
        error = trg-curr;
        integral += error*sens::dt;
        derivitive = (error-error0)/sens::dt;
    }
    inline double get_val(double trg, double curr, double KP, double KI, double KD) {
        update(trg, curr);
        return error*KP + integral*KI + derivitive*KD;
    }
    inline tuple<double,double,double> get_seperate(double trg, double curr, double KP, double KI, double KD) {
        update(trg, curr);
        return {error*KP, integral*KI, derivitive*KD};
    }
};