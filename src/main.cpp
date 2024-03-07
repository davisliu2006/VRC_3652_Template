#include "globals.hpp"
#include "display/main.hpp"
#include "lib/autonomous.hpp"
#include "lib/opcontrol.hpp"
#include "route/route.hpp"

#if __has_include("test/auton_test.hpp")
#include "test/auton_test.hpp"
#endif

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // IMPORTANT: initialize all pneumatics to 0

    // lcd
    display::init_all();
    display::on_init();

    // drivetrain
    flmotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    frmotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rlmotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rrmotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    flmotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    frmotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    rlmotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    rrmotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    WHEEL_RPM = gear_mp[flmotor.get_gearing()];
    WHEEL_RPS = WHEEL_RPM/60;
    WHEEL_LSPD = WHEEL_RPS*WHEEL_C;
    cout << "WHEEL_RPM: " << WHEEL_RPM << '\n';

    // STALLING INITIALIZATIONS
    sens::reset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    display::on_disable();
    sens::reset();
    auton::need_sens_reset = false;
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    display::on_init();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    sens::ROT_OFFSET = 0;
    sens::rot_trg = sens::ROT_OFFSET;
    display::on_auton();
    auton::init();
    static vector<function<void()>> route_mp = {
        
    };
    if (selection::route < route_mp.size()) {
        route_mp[selection::route]();
    }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    #ifdef AUTON_TEST // auton test
    selection::route = AUTON_TEST;
    autonomous();
    #endif

    display::on_opcontrol();
    if (!auton::did_init) {auton::init();}
    auton::need_sens_reset = true;
    opcontrol_start();
} 