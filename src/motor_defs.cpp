#include "main.h"
#include  "motor_defs.hpp"

MotorDefs::MotorDefs() {
    left_mtr_t = new pros::Motor(16, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    left_mtr_b = new pros::Motor(19, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    right_mtr_t = new pros::Motor(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    right_mtr_b = new pros::Motor(13, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

    intake_r = new pros::Motor(10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    intake_l = new pros::Motor(8, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    roller_t = new pros::Motor(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    roller_b = new pros::Motor(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
}

MotorDefs::~MotorDefs() {
    delete left_mtr_b;
    delete left_mtr_t;
    delete right_mtr_t;
    delete right_mtr_b;
    delete intake_l;
    delete intake_r;
    delete roller_t;
    delete roller_b;
}
