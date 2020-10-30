//motor definitions for full robot
#include "main.h"
#ifndef _MOTORDEFS_H_
#define _MOTORDEFS_H_

class MotorDefs {
    public:
        pros::Motor *left_mtr_t;
        pros::Motor *left_mtr_b;
        pros::Motor *right_mtr_t;
        pros::Motor *right_mtr_b;
        pros::Motor *intake_l;
        pros::Motor *intake_r;
        pros::Motor *roller_t;
        pros::Motor *roller_b;

        MotorDefs();
        ~MotorDefs();
};


#endif //_MOTORDEFS_H_