#include "motordefs.hpp"

const int LEFT_MTR_B_PORT = 19;
const int LEFT_MTR_T_PORT = 16;
const int RIGHT_MTR_B_PORT = 13;
const int RIGHT_MTR_T_PORT = 2;
const int LEFT_INTAKE_MTR_PORT = 8;
const int RIGHT_INTAKE_MTR_PORT = 10;
const int TOP_ROLLER_MTR_PORT = 5;
const int BOTTOM_ROLLER_MTR_PORT = 3;

MotorDefs::MotorDefs() {
    left_mtr_t = new pros::Motor(LEFT_MTR_T_PORT, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    right_mtr_t = new pros::Motor(RIGHT_MTR_T_PORT, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    left_mtr_b = new pros::Motor(LEFT_MTR_B_PORT, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    right_mtr_b = new pros::Motor(RIGHT_MTR_B_PORT, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    roller_t = new pros::Motor(TOP_ROLLER_MTR_PORT, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
    roller_b = new pros::Motor(BOTTOM_ROLLER_MTR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    intake_l = new pros::Motor(LEFT_INTAKE_MTR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    intake_r = new pros::Motor(RIGHT_INTAKE_MTR_PORT, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

    //left_mtr_b->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES)
}

MotorDefs::~MotorDefs() { 
    delete left_mtr_b;
    delete left_mtr_t;
    delete right_mtr_t;
    delete right_mtr_b;
    delete roller_b;
    delete roller_t;
    delete intake_l;
    delete intake_r;  
} 

